#ifndef KUKADU_TICTOC
#define KUKADU_TICTOC

#include <vector>
#include <string>

class TicToc {

private:

    std::vector<timeval> ticList;
    std::vector<timeval> tocList;

    std::vector<std::string> strList;

public:

    void tic(std::string str);
    double toc(std::string str);

};


#endif
