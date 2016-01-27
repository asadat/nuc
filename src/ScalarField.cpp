#include "ScalarField.h"
#include "GL/glut.h"

using namespace std;

shared_ptr<ScalarField> ScalarField::instance;

ScalarField::ScalarField():GaussianProcess(2, "CovSum ( CovSEiso, CovNoise)")
{
    Eigen::VectorXd params(covf().get_param_dim());
    params << 4.0, 1.5, -0.7;
    covf().set_loghyper(params);
}

ScalarField::~ScalarField()
{

}

void ScalarField::glDraw()
{
    auto s = this->sampleset->size();
    glPointSize(5);
    glColor3f(1,0.5,0);
    glBegin(GL_POINTS);
    for(size_t i=0; i<s; i++)
    {
        auto x = this->sampleset->x(i);
        glVertex3f(x[0], x[1], 1);
    }
    glEnd();
}
