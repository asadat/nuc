#pragma once
#include <memory>
#include "gp/gp.h"

class ScalarField: public libgp::GaussianProcess
{
public:
    static std::shared_ptr<ScalarField> GetInstance()
    {
        if(!instance)
            instance =  std::shared_ptr<ScalarField>(new ScalarField());

        return instance;
    }

    virtual ~ScalarField();

private:
    static std::shared_ptr<ScalarField> instance;
    ScalarField();
};
