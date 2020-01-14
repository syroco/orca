/*
 * Copyright (C) 2014,2018 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 *
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#include <iDynTree/Controller.h>

namespace iDynTree {
    namespace optimalcontrol {

    Controller::Controller(size_t controlSpaceSize)
    :m_controllerSize(controlSpaceSize)
    {}

    Controller::~Controller()
    {}

    bool Controller::setStateFeedback(double time, const VectorDynSize &stateFeedback){
        return false;
    }

    size_t Controller::controlSpaceSize(){
        return m_controllerSize;
    }

    }
}