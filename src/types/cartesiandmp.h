#ifndef CARTESIANDMP_H
#define CARTESIANDMP_H

#include "DMP.h"

class CartesianDmp : public Dmp {
public:
    CartesianDmp();

    bool isCartesian();

    std::shared_ptr<Trajectory> copy();
};

#endif // CARTESIANDMP_H
