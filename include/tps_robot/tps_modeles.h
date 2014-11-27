#include <visp/vpHomogeneousMatrix.h>

class ModelGeom
{
public:
    inline ModelGeom(const std::string &_name)
    {
        robot = _name;
    }

    int calcMGD(const vpColVector &q, vpColVector &pose);

    int calcJac(const vpColVector &q, vpMatrix &J);

    int calcMGI(const vpColVector &q0, const vpColVector &pose_des, vpColVector &q);

    int calcJacobian(const vpColVector &q, vpMatrix &J);

private:
    std::string robot;

};
