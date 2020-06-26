#ifndef LINE_H
#define LINE_H
#include <vector>

class Line
{
public:
    Line();
    Line(double gradient, double y_intercept);
    Line(double ax, double ay, double bx, double by);
    void fromPoints(double ax, double ay, double bx, double by);
    void setGradient(double gradient);
    double getGradient();
    void setYIntercept(double y_intercept);
    double getYIntercept();
    bool pointAboveLine(double x, double y);
    void setEquations(double ax, double ay, double bx, double by);
    std::vector<double> getEquations();
private:
    double gradient_;
    double y_intercept_;
    std::vector<double> equations_;
};

#endif // LINE_H
