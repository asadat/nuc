#include "ScalarField.h"
using namespace std;

shared_ptr<ScalarField> ScalarField::instance;

ScalarField::ScalarField():GaussianProcess(2, "CovSum ( CovSEiso, CovNoise)")
{
    Eigen::VectorXd params(covf().get_param_dim());
    params << 2.0, 0.0, -2.0;
    covf().set_loghyper(params);
}

ScalarField::~ScalarField()
{

}
