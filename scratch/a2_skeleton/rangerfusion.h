/**
  * @file       rangerfusion.h
  * @defgroup   Fusion Fusion
  * @brief      Fuses cell data with Ranger data
  * @author     Ajal Singh
  * @version    1.0
  * @date       April 2020
  */

#ifndef RANGERFUSION_H
#define RANGERFUSION_H

#include <vector>
#include "rangerfusioninterface.h"
#include "rangerinterface.h"

//_____________________________________________________________________________ Struct Definition

/**
 * @brief   The Point struct.
 *          Creates points with x and y coordinates
 */
struct Point{
    double x;   //!<  x coordinate
    double y;   //!<  y coordinate
};

//_____________________________________________________________________________ Class Definition

/**
 * @class   RangerFusion
 * @brief   The RangerFusion class is a child of abstract class RangerFusionInterface
 *          Accepts cells and rangers and fuses the data.
 *          Checks if the ranger data occupies a cell, or passes through the cell (free).
 *          Cell state is then updated.
 */
class RangerFusion: public RangerFusionInterface
{
//_____________________________________________________________________________ Class Public Members
public:
  //Default constructor should set all
  //RangerFusion attributes to a default value
  /**
   * @brief RangerFusion default constructor
   */
  RangerFusion();

  /**
   * @brief RangerFusion default destructor
   */
  ~RangerFusion();

  //See rangerfusioninterface.h for more information
  // Accepts container of rangers - as per requirement C1
  /**
   * @brief Member function prepares ranger data for fusion
   * @param rangers
   */
  void setRangers(std::vector<RangerInterface*> rangers);

  // Accepts container of cells - as per requirement C2
  /**
   * @brief Member function prepares cells data for fusion
   * @param cells - vector of pointers
   */
  void setCells(std::vector<Cell*> cells);

  // Grab data and fuse - as per requirement C3
  /**
   * @brief Member function checks if the ranger data occupies a cell, or passes through the cell (free).
   *        Cell state is then updated, OCCUPIED (-1), FREE (1), and UNKNOWN(0)
   */
  void grabAndFuseData();

  // Returns a container of raw data range readings - as per requirement C5
  /**
   * @brief     Member function gets raw range data generated from ranger class which was used in fusion
   * @return    Raw Data for rangers
   */
  std::vector<std::vector<double>> getRawRangeData();

  /**
   * @brief     Member function gets cells with updated cell states after fusion
   * @return    Cell object pointers
   */
  std::vector<Cell*> getCells(void);

//_____________________________________________________________________________ Class Private Members
private:
  /**
   * @brief     Member function calculates Cell Geometry
   * @details   Uses cell's centre x and y coordinates to calculate:
   *            \n 1) the x and y boundaries
   *            \n 2) the locations (x and y coordinate) of the four corners
   * @param     cell object
   */
  void calculateCellGeometry(Cell* cell);

  /**
   * @brief     Member function checks if the laser point provided, has any interaction with cell
   * @param     data_x: x coordinate of laser data point
   * @param     data_y: y  of laser data point
   * @return    OCCUPIED: -1
   *            \n FREE: 1 (Laser passes through cell)
   *            \n UNKNOWN: 0
   */
  int analyseLaser(double data_x, double data_y);

  /**
   * @brief     Member function checks if the sonar data provided, has any interaction with cell
   * @details   Sonar is represented by a triangle where 2 points are provided as params, the third point is the origin (0,0)
   * @param     sonar_p1
   * @param     sonar_p2
   * @return    OCCUPIED: -1
   *            \n FREE: 1 (Sonar passes through cell)
   *            \n UNKNOWN: 0
   */
  int analyseSonar(Point sonar_p1, Point sonar_p2);

  /**
   * @brief     Member function checks if two given lines intersect
   * @details   P1 and P2 represent endpoints of line 1, P2 and P3 represent endpoints of Line 2
   * @param     p1: Line 1 Point 1
   * @param     p2: Line 1 Point 2
   * @param     p3: Line 2 Point 1
   * @param     p4: Line 2 Point 2
   * @return    returns true if lines intersect
   */
  bool checkForIntersect(Point p1, Point p2, Point p3, Point p4);

  /**
   * @brief     Member function determines the orientation of ordered triplet (p,q,r)
   * @param     p
   * @param     q
   * @param     r
   * @note      p, q, r are of Point type
   * @return    0, 1, or 2. Colinear, clockwise or anti-clockwise respectively
   * @sa        See https://www.geeksforgeeks.org/orientation-3-ordered-points/
   */
  double orientation(Point p, Point q, Point r);

  /**
   * @brief     Member function given three colinear points p, q, r, the function checks
   *            if point q lies on line segment 'pr'
   * @param     p
   * @param     q
   * @param     r
   * @note      p, q, r are of Point type
   * @return    True when Point q lies on segment pr
   */
  bool checkOnSegment(Point p, Point q, Point r);

  /**
   * @brief     Member function calculates area of triangle formed by Point p1, Point p2, and Point p3
   * @param     p1
   * @param     p2
   * @param     p3
   * @return    Triangle area
   */
  double triangleArea(Point p1, Point p2, Point p3);


  //This is to cater for getRawRangeData (which generates the raw data))
  std::vector<std::vector<double>> data_;           //!<    Holds raw ranger data

  std::vector<RangerInterface*> rangers_;           //!<    RangerInterface Class object pointers
  std::vector<Cell*> cells_;                        //!<    Cell class object pointers

  /**
   * @brief Holds generated data of rangers.
   *        \n Every even element (0,2,...) holds a vector of angles.
   * @note Angles are scanned anti-clockwsie from the positive x-axis.
   */
  std::vector<std::vector<double>> angle_for_fusion_;

  std::vector<double> cell_side_coordinates_;       //!<    Holds upper and lower boundaries (x and y) of a cell
  std::vector<Point> cell_points_;                  //!<    Holds the four corners of a cell as points
};

#endif // RANGERFUSION_H
