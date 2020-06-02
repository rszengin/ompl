/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isStateValid(const ob::State *state)
{
    // cast the abstract state type to the type we expect
    const auto *dubins_state = state->as<ob::DubinsStateSpace::StateType>();

    // extract the first component of the state and cast it to what we expect
    const auto *pos = dubins_state->as<ob::RealVectorStateSpace::StateType>(0);

    // extract the second component of the state and cast it to what we expect
    const auto *rot = dubins_state->as<ob::SO2StateSpace::StateType>(1);

    // check validity of state defined by pos & rot


    // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
    return (const void*)rot != (const void*)pos;
}

void planWithSimpleSetup()
{
    // construct the state space we are planning in
    auto space(std::make_shared<ob::DubinsStateSpace>(0.1, false));

    // set the bounds for the R^2 part of SE(2)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1);
    bounds.setHigh(1);

    space->setBounds(bounds);

    // define a simple setup class
    og::SimpleSetup ss(space);
    
    ss.setPlanner(std::make_shared<og::RRTstar>(ss.getSpaceInformation()));

    // set state validity checking for this space
    ss.setStateValidityChecker([](const ob::State *state) { return isStateValid(state); });

    // create a random start state
    ob::ScopedState<> start(space);
    start = {-0.95, -0.95, 0};
    // start.print();

    // create a random goal state
    ob::ScopedState<> goal(space);
    goal = {0.95, 0.95, 0};
    // goal.print();

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);

    // this call is optional, but we put it in to get more output information
    ss.setup();
    ss.print();

    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = ss.solve(5.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        // ss.simplifySolution();
        ss.getSolutionPath().print(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
}

int main(int /*argc*/, char ** /*argv*/)
{
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    planWithSimpleSetup();

    return 0;
}
