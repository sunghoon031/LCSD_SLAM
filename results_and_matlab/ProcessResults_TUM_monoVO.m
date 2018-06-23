
for system = systems_to_evaluate_TUM_monoVO
    if (system == 1)
         [ OUR_VO_RESULTS_TUM_monoVO ] = evalLOOPdataset(FILEPATH_gt, FILEPATH_our_vo, 'our_vo');
        save('OUR_VO_RESULTS_TUM_monoVO');
    elseif (system == 2)
        [ OUR_SLAM_RESULTS_TUM_monoVO ] = evalLOOPdataset(FILEPATH_gt, FILEPATH_our_slam, 'our_slam');
        save('OUR_SLAM_RESULTS_TUM_monoVO');
    elseif (system == 3)
        [ DSO_DEFAULT_RESULTS_TUM_monoVO ] = evalLOOPdataset(FILEPATH_gt, FILEPATH_dso_default, 'dso_default');
        save('DSO_DEFAULT_RESULTS_TUM_monoVO');
    elseif (system == 4)
        [ DSO_REDUCED_RESULTS_TUM_monoVO ] = evalLOOPdataset(FILEPATH_gt, FILEPATH_dso_reduced, 'dso_reduced');
        save('DSO_REDUCED_RESULTS_TUM_monoVO');
    elseif (system == 5)
        [ ORB_VO_RESULTS_TUM_monoVO ] = evalLOOPdataset(FILEPATH_gt, FILEPATH_orb_vo, 'orb_vo');
        save('ORB_VO_RESULTS_TUM_monoVO');
    elseif (system == 6)
        [ ORB_SLAM_RESULTS_TUM_monoVO ] = evalLOOPdataset(FILEPATH_gt, FILEPATH_orb_slam, 'orb_slam');
        save('ORB_SLAM_RESULTS_TUM_monoVO');
    end
end



