#include "rangerfusion.h"

#include <chrono>
#include <random>
#include <cmath>
#include <math.h>
#include <algorithm>
using std::max;
using std::min;

RangerFusion::RangerFusion(){

}

RangerFusion::~RangerFusion(){}

// Accepts container of rangers - as per requirement C1
void RangerFusion::setRangers(std::vector<RangerInterface*> rangers){
    angle_for_fusion_.clear();
    data_.clear();

    for (auto ranger : rangers){
        std::vector<double> ranger_data;
        std::vector<double> ranger_data_degrees;

        ranger_data = ranger->generateData();

        //determine the angle at which the laser is pointed
        if (ranger->getSensingMethod() == POINT){
            for (int i=0;i<ranger_data.size();i++){
                if (i==0){
                    ranger_data_degrees.push_back(i + ranger->getOffset());
                }
                else{
                    double new_index = ranger_data_degrees[i-1] + ranger->getAngularResolution();
                    ranger_data_degrees.push_back(new_index);
                }
            }
            angle_for_fusion_.push_back(ranger_data_degrees);
        }

        //if sensor type is cone
        else{
            double index = ranger->getOffset();
            ranger_data_degrees.push_back(index);
            angle_for_fusion_.push_back(ranger_data_degrees);
        }
        data_.push_back(ranger_data);
    }
}

// Accepts container of cells - as per requirement C2
void RangerFusion::setCells(std::vector<Cell*> cells){
    cells_ = cells;
}

// Grab data and fuse - as per requirement C3
void RangerFusion::grabAndFuseData(){
    for (auto cell : cells_){

        calculateCellGeometry(cell);

        for (int i=0;i<angle_for_fusion_.size();i++){

            //laser
            if (angle_for_fusion_[i].size() > 1){
                for (int j=0;j<angle_for_fusion_[i].size();j++){
                    double laser_data_x, laser_data_y;

                    if (angle_for_fusion_[i][j] <= 90 && angle_for_fusion_[i][j] >= 0){
                        laser_data_x = data_[i][j] * cos(angle_for_fusion_[i][j] * M_PI/180);
                        laser_data_y = data_[i][j] * sin(angle_for_fusion_[i][j] * M_PI/180);
                    }
                    else if (angle_for_fusion_[i][j] > 90 && angle_for_fusion_[i][j] <= 180){
                        laser_data_x = -(data_[i][j] * cos(M_PI - angle_for_fusion_[i][j] * M_PI/180));
                        laser_data_y = data_[i][j] * sin(M_PI - angle_for_fusion_[i][j] * M_PI/180);
                    }

                    //check for cell interaction
                    int result = analyseLaser(laser_data_x, laser_data_y);

                    if (result == -1){
                        cell->setState(OCCUPIED);
                        break;
                    }
                    else if (result == 1 && cell->getState() == UNKNOWN){
                        cell->setState(FREE);
                    }
                }
            }
            //sonar
            else{
                const int j = 0;
                int result;
                double sonar_dist, sonar_side_dist, angle;
                Point sonar_p1, sonar_p2;

                //Sonar Geometry
                sonar_dist = data_[i][j];
                sonar_side_dist = sonar_dist / cos(10 * M_PI/180);
                angle = angle_for_fusion_[i][j] + 90;

                sonar_p1.x = sonar_side_dist * cos((angle+10)* M_PI/180);
                sonar_p1.y = sonar_side_dist * sin((angle+10)* M_PI/180);
                sonar_p2.x = sonar_side_dist * cos((angle-10)* M_PI/180);
                sonar_p2.y = sonar_side_dist * sin((angle-10)* M_PI/180);

                //check for cell interaction
                result = analyseSonar(sonar_p1, sonar_p2);

                if (result == -1){
                    cell->setState(OCCUPIED);
                    break;
                }
                else if (result == 1 && cell->getState() == UNKNOWN){
                    cell->setState(FREE);
                }
            }
        }
    }
}

// Returns a container of raw data range readings - as per requirement C5
std::vector<std::vector<double>> RangerFusion::getRawRangeData(){
    return data_;
}

std::vector<Cell*> RangerFusion::getCells(void){
    return cells_;
}

void RangerFusion::calculateCellGeometry(Cell* cell){
    double cell_low_x, cell_low_y, cell_high_x, cell_high_y, cell_centre_x, cell_centre_y;
    Point cell_btm_left, cell_btm_right, cell_up_left, cell_up_right, origin;

    cell->setState(UNKNOWN);
    cell->getCentre(cell_centre_x,cell_centre_y);

    //get cell sides
    cell_low_x = cell_centre_x - cell->getSide()/2;
    cell_low_y = cell_centre_y - cell->getSide()/2;
    cell_high_x = cell_centre_x + cell->getSide()/2;
    cell_high_y = cell_centre_y + cell->getSide()/2;

    //get cell points
    cell_btm_left = {cell_low_x, cell_low_y};
    cell_btm_right = {cell_high_x,cell_low_y};
    cell_up_left = {cell_low_x, cell_high_y};
    cell_up_right = {cell_high_x, cell_high_y};
    origin = {0,0};

    //push sides and points into vectors
    cell_side_coordinates_ = {cell_low_x, cell_low_y, cell_high_x, cell_high_y};
    cell_points_ = {cell_btm_left, cell_btm_right, cell_up_left, cell_up_right, origin};
}

int RangerFusion::analyseLaser(double laser_data_x, double laser_data_y){
    //check if cell occupied
    if (laser_data_x >= cell_side_coordinates_[0] && laser_data_x <= cell_side_coordinates_[2] && laser_data_y >= cell_side_coordinates_[1] && laser_data_y <= cell_side_coordinates_[3]){
        return -1; //occupied
    }

    //if not occupied, check free
    else{
        Point laser_line = {laser_data_x, laser_data_y};
        double laser_line_bottom, laser_line_left, laser_line_right, laser_line_top;
        std::vector<double> laser_cell_intersections;
        int intersections_count = 0;

        laser_line_bottom = checkForIntersect(laser_line, cell_points_[4], cell_points_[0], cell_points_[1]);
        laser_line_left = checkForIntersect(laser_line, cell_points_[4], cell_points_[0], cell_points_[2]);
        laser_line_right = checkForIntersect(laser_line, cell_points_[4], cell_points_[1], cell_points_[3]);
        laser_line_top = checkForIntersect(laser_line, cell_points_[4], cell_points_[2], cell_points_[3]);

        laser_cell_intersections = {laser_line_bottom, laser_line_left, laser_line_right, laser_line_top};

        for (int i=0;i<laser_cell_intersections.size();i++){
            if (laser_cell_intersections[i] == 1){
                intersections_count++;
            }
            //if 2 or more intersections occur, state must be FREE
            if (intersections_count >= 2){
                return 1;
            }
        }
    }
}

int RangerFusion::analyseSonar(Point sonar_p1, Point sonar_p2){
    bool bottom, left, right, top;
    const Point C = {0,0};

    bottom = checkForIntersect(sonar_p1, sonar_p2, cell_points_[0], cell_points_[1]);
    left = checkForIntersect(sonar_p1, sonar_p2, cell_points_[0], cell_points_[2]);
    right = checkForIntersect(sonar_p1, sonar_p2, cell_points_[1], cell_points_[3]);
    top = checkForIntersect(sonar_p1, sonar_p2, cell_points_[2], cell_points_[3]);

    //check if cell occupied
    if (bottom == 1 || left == 1 || right == 1 || top == 1){
        return -1; //occupied
    }

    //cell not occupied, check if free
    else{
        const Point origin = cell_points_.back();
        cell_points_.pop_back();
        for (auto point:cell_points_){

           double A = triangleArea (sonar_p1, sonar_p2, C);       //area of triangle ABC
           double A1 = triangleArea (point, sonar_p2, C);         //area of PBC
           double A2 = triangleArea (sonar_p1, point, C);         //area of APC
           double A3 = triangleArea (sonar_p1, sonar_p2, point);       //area of ABP

           if (A == A1 + A2 + A3){
               return 1; //free
           }
        }
        cell_points_.push_back(origin);
    }
}

bool RangerFusion::checkForIntersect(Point p1, Point p2, Point p3, Point p4){
    //check orientation of triplet
    double o1 = orientation(p1, p2, p3);
    double o2 = orientation(p1, p2, p4);
    double o3 = orientation(p3, p4, p1);
    double o4 = orientation(p3, p4, p2);

    // General case
    if (o1 != o2 && o3 != o4)
        return true;

    // Special Cases
    // p1, p2 and p3 are colinear and p3 lies on segment p1p2
    if (o1 == 0 && checkOnSegment(p1, p3, p2)) return true;

    // p1, p2 and p4 are colinear and p4 lies on segment p1p2
    if (o2 == 0 && checkOnSegment(p1, p4, p2)) return true;

    // p3, p4 and p1 are colinear and p1 lies on segment p3p4
    if (o3 == 0 && checkOnSegment(p3, p1, p4)) return true;

     // p3, p4 and p2 are colinear and p2 lies on segment p3p4
    if (o4 == 0 && checkOnSegment(p3, p2, p4)) return true;

    return false; // Doesn't fall in any of the above cases, i.e no intersect
}

double RangerFusion::orientation(Point p, Point q, Point r)
{
    double val = (q.y - p.y) * (r.x - q.x) -
              (q.x - p.x) * (r.y - q.y);

    if (val == 0) return 0;  // colinear

    return (val > 0)? 1: 2; // clock or counterclock wise
}

bool RangerFusion::checkOnSegment(Point p, Point q, Point r)
{
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) &&
        q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y))
       return true;

    return false;
}


double RangerFusion::triangleArea(Point p1, Point p2, Point p3)
{
    //find area of triangle formed by p1, p2 and p3
    double area = (p1.x*(p2.y-p3.y) + p2.x*(p3.y-p1.y)+ p3.x*(p1.y-p2.y))/2.0;
    return fabs(area);
}
