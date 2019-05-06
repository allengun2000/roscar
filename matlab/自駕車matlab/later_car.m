function steerCmd = later_car(refPose, currPose, currVelocity, direction ,gain ,wheelbase,maxSteer)


narginchk(3, 11);

% Parse and check inputs


% Clip heading angle to be within [0, 360) and convert to radian
refPose(3) = matlabshared.planning.internal.angleUtilities.convertAndWrapTo2Pi(refPose(3));
currPose(3)= matlabshared.planning.internal.angleUtilities.convertAndWrapTo2Pi(currPose(3));

% Compute position error
posError = computePositionError(refPose, currPose, direction, wheelbase);

% Compute heading error
angError = computeSteeringAngleError(refPose, currPose);

% Avoid oversensitiveness at low speeds
Ksoft = 1;

% Compute steering angle by implementing equation (5) in reference [1]
if direction == 1
    delta = -(angError + atan(gain * posError/(Ksoft+currVelocity)));
else
    % In reverse motion, rear wheel velocity is required to compute the
    % steering angle. However, this will require the current steering angle 
    % as an input. To simplify the design without affecting the performance
    % of the controller, the front wheel velocity is used.
    vr    = currVelocity;
    delta = angError + atan(gain * posError/(-Ksoft+vr));
end

% Saturate and convert to degree
steerCmd = saturate(rad2deg(delta), maxSteer);

end

%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
% Helper function
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
function delta = saturate(delta, maxSteeringAngle)
%saturate Saturate steering angle when it exceeds MaxSteeringAngle
delta = sign(delta) * min(abs(delta), maxSteeringAngle);
end

%--------------------------------------------------------------------------
function thetaError = computeSteeringAngleError(refPose, pose)
%computeSteeringAngleError Compute the steering angle error
thetaError = matlabshared.planning.internal.angleUtilities.angdiff(refPose(3), pose(3));

end

%--------------------------------------------------------------------------
function posError = computePositionError(refPose, pose, direction, wheelbase)
%computePositionError Compute the distance error

% Tangent direction of the reference path
tHat = [cos(refPose(3)), sin(refPose(3))];

% Tracking error vector
if direction == 1
    poseF = driving.internal.control.rearPoseToFrontPose(pose, wheelbase);
    d = poseF(1:2) - refPose(1:2);
else
    d = pose(1:2)  - refPose(1:2);
end

% Implement equation (8) in reference [2]
posError = -(d(1)*tHat(2) - d(2)*tHat(1));
end

