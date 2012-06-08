VisualISAMGlobalVars

%% Initialize iSAM
isam = visualSLAMISAM(REORDER_INTERVAL);
newFactors = visualSLAMGraph;
initialEstimates = visualSLAMValues;
i1 = symbol('x',1);
camera1 = cameras{1};
pose1 = camera1.pose;
if HARD_CONSTRAINT % add hard constraint
    newFactors.addPoseConstraint(i1,pose1);
else
    newFactors.addPosePrior(i1,pose1, poseNoise);
end
initialEstimates.insertPose(i1,pose1);
% Add visual measurement factors from first pose
for j=1:nPoints
    jj = symbol('l',j);
    if POINT_PRIORS % add point priors
        newFactors.addPointPrior(jj, points{j}, pointNoise);
    end
    zij = camera1.project(points{j});
    newFactors.addMeasurement(zij, measurementNoise, i1, jj, K);
    initialEstimates.insertPoint(jj, points{j});
end

frame_i = 1
result = initialEstimates;
cla;