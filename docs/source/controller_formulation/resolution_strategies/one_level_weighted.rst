.. _one_level_weighted:


*******************************
One Level Weighted
*******************************

.. code-block:: c++
    case ResolutionStrategy::OneLevelWeighted:
    {
        updateTasks(current_time,dt);
        updateConstraints(current_time,dt);
        auto problem = getProblemAtLevel(0);
        problem->build();
        solution_found_ = problem->solve();

        if(this->update_cb_)
            this->update_cb_(current_time,dt);

        static bool print_warning = true;
        if(solution_found_ && isProblemDry(problem) && print_warning)
        {
            print_warning = false;
            LOG_WARNING << "\n\n"
                <<" Solution found but the problem is dry !\n"
                << "It means that an optimal solution is found but the problem \n"
                << "only has one task computing anything, ans it's the"
                << "GlobalRegularisation task (This will only be printed once)\n\n"
                << "/!\\ Resulting torques will cause the robot to fall /!\\";
        }

        return solution_found_;
    }
