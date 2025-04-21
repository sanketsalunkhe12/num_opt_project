#include "plot.h"

namespace plt = matplotlibcpp;

void plot() {
    std::vector<double> x, y, z;
    for (double t = 0; t < 10; t += 0.1) {
        x.push_back(t);
        y.push_back(sin(t));
        z.push_back(cos(t));
    }

    plt::figure();
    plt::plot3(x, y, z); // requires matplotlib >= 3.2
    plt::show();
}
