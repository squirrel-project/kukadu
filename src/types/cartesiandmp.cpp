#include "cartesiandmp.h"

CartesianDmp::CartesianDmp() {
}

bool CartesianDmp::isCartesian() {
    return true;
}

std::shared_ptr<Trajectory> CartesianDmp::copy() {

    return std::shared_ptr<Trajectory>(new CartesianDmp(*this));

}
