%HelperCubicSplineFit Fit a cubic spline through waypoints.
%   This is a helper class that generates a cubic parametric spline which
%   goes through all the waypoints stored in a driving.path object. Use fit 
%   method to retrieve the interpolated value, the first and the second 
%   derivatives of query points on the spline.
%
%   NOTE: The name of this class and it's functionality may change without
%   notice in a future release, or the class itself may be removed.
%   
%   splineFitter = HelperCubicSplineFit(refPath) returns a object for
%   spline fitting. refPath is a driving.path object storing the poses of
%   the vehicle at the waypoints.
%
%   splineFitter = HelperCubicSplineFit(refPath, t) also specifies the
%   variable t that parameterizes x and y position in waypoints.
%
%
%   HelperCubicSplineFit properties:
%   X                    - X position of the vehicle
%   Y                    - Y position of the vehicle
%   Theta                - Heading angle of the vehicle (in degree)
%   t                    - Variable parameterizes X and Y
%   ConstrainOrientation - Flag indicating matching end first derivatives
%                          in fitting the spline
%                          
%
%   HelperCubicSplineFit methods:
%   fit                  - Fit a spline and return interpolated values
%                          for a given number of query points
%   
%
%   See also driving.Path
%

% Copyright 2017 The MathWorks, Inc.

classdef HelperCubicSplineFit < handle
    
    properties 
        %X X-coordinate of vehicle
        X
        
        %Y Y-coordinate of vehicle
        Y
        
        %Theta Orientation of vehicle
        Theta
        
        %t Variable parameterizes X and Y
        t
        
        %ConstrainOrientation Matching end first derivatives
        ConstrainOrientation = true
    end
    
    properties (Access = private, Hidden)
        %PPoly Piecewise polynomial structure of X spline
        XPPoly
        
        %PPoly Piecewise polynomial structure of Y spline
        YPPoly
    end
    
    properties (Access = private, Dependent)
        %d1XPPoly First derivative of X (Piecewise polynomial structure)
        d1XPPoly
        
        %d2XPPoly Second derivative of X (Piecewise polynomial structure)
        d2XPPoly
        
        %d1YPPoly First derivative of Y (Piecewise polynomial structure)
        d1YPPoly
        
        %d2YPPoly Second derivative of Y (Piecewise polynomial structure)
        d2YPPoly   
    end
    

    methods
        %------------------------------------------------------------------
        function obj = HelperCubicSplineFit(varargin)
            
            narginchk(0, 2);
             
            if nargin == 0
                return;
            else
                obj.setPoints(varargin{:});
            end
            
        end
        
        %------------------------------------------------------------------
        function setPoints(obj, refPath, t)
             % Validate reference path
             validateattributes(refPath, {'driving.Path'}, {'scalar'}, ...
                 'refPath', mfilename);
             poses = refPath.KeyPoses;
             
             % Validate t
             if nargin < 3
                 t = (1 : size(poses,1))';
             end
             validateattributes(t, {'single','double'}, ...
                 {'vector','nrows', size(poses,1),'nonempty','increasing'}, ...
                 't', mfilename);
             
             if size(poses,1) >= 4
                 obj.t       = t;
                 obj.X       = poses(:, 1);
                 obj.Y       = poses(:, 2);
                 obj.Theta   = poses(:, 3);
             else
                 obj.resamplePoses(refPath, t);
             end
        end
        
        %------------------------------------------------------------------
        function splineData = fit(obj, numPoints)
            %fit fit the spline and returns interpolated values for a given 
            %   number of query points numPoints. splineData is a struct
            %   stores the interpolated value, the first and the second
            %   derivatives at the query points.
            
            validateattributes(numPoints, {'single', 'double'}, ...
                {'scalar','>', 1, 'finite', 'integer'}, 'numPoints', ...
                mfilename);
            
            % Calculate the coefficients of x and y splines
            obj.computeCoef(); 
            
            ts = linspace(1, obj.t(end), numPoints)';
            
            % Interpolate x and y splines
            [x, dx, ddx] = obj.interp(ts, 'X');
            [y, dy, ddy] = obj.interp(ts, 'Y');
            
            % Curvature
            kappa = abs(dx.*ddy - dy.*ddx)./(dx.^2 + dy.^2).^(3/2);
            
            % Assignment
            splineData.ts    = ts;
            splineData.x     = x;
            splineData.y     = y;
            splineData.dx    = dx;
            splineData.dy    = dy;
            splineData.kappa = kappa;
        end        
    end
    
    %----------------------------------------------------------------------
    % Hidden methods
    %----------------------------------------------------------------------
    methods (Access = private, Hidden)
        %------------------------------------------------------------------
        function computeCoef(obj)
            %computePP Compute coefficients of x and y splines
            
            if obj.ConstrainOrientation % End first derivative constraints
                disStart = sqrt((obj.X(2)-obj.X(1))^2+(obj.Y(2)-obj.Y(1))^2)/(obj.t(2) - obj.t(1));
                disEnd   = sqrt((obj.X(end)-obj.X(end-1))^2+(obj.Y(end)-obj.Y(end-1))^2)/(obj.t(end) - obj.t(end-1));
                
                obj.XPPoly = spline(obj.t, [disStart*cosd(obj.Theta(1)); obj.X; disEnd*cosd(obj.Theta(end))]);
                obj.YPPoly = spline(obj.t, [disStart*sind(obj.Theta(1)); obj.Y; disEnd*sind(obj.Theta(end))]);               
            else
                obj.XPPoly = spline(obj.t, obj.X);
                obj.YPPoly = spline(obj.t, obj.Y);
            end              
        end
        
        %------------------------------------------------------------------
        function [p, dp, ddp] = interp(obj, ts, curveName)
            % interp Evaluate the spline for query points in ts
            
            if strcmp(curveName, 'X')
                p   = ppval(obj.XPPoly,   ts);
                dp  = ppval(obj.d1XPPoly, ts);
                ddp = ppval(obj.d2XPPoly, ts);
            elseif strcmp(curveName, 'Y')
                p   = ppval(obj.YPPoly,   ts);
                dp  = ppval(obj.d1YPPoly, ts);
                ddp = ppval(obj.d2YPPoly, ts);
            end
        end
        
        %------------------------------------------------------------------
        function resamplePoses(obj, refPath, t)
            %resamplePoses Resample the raw path data to have enough 
            %   number of waypoints to fit cubic spline
            
            if size(refPath.KeyPoses, 1) == 2 % 2 waypoints
                innerPoses = [refPath.KeyPoses(1,:); extractSegment(refPath, 1)];
            else % 3 waypoints
                innerPoses = [refPath.KeyPoses(1,:); 
                              extractSegment(refPath, 1);
                              extractSegment(refPath, 2)];
            end
            
            % Resample to have 4 waypoints
            poses = innerPoses([1 floor(end/3) floor(end*2/3) end], :);
            
            obj.t       = [1 0; 2/3 1/3; 1/3 2/3; 0 1] * [t(1); t(end)];
            obj.X       = poses(:, 1);
            obj.Y       = poses(:, 2);
            obj.Theta   = poses(:, 3);
        end
    end
    
    %----------------------------------------------------------------------
    % Accessors
    %----------------------------------------------------------------------
    methods
        %------------------------------------------------------------------
        function d1pp = get.d1XPPoly(obj)
            n = length(obj.X);
            d1Coefs = obj.XPPoly.coefs(:, 1:3).* repmat([3 2 1], [n-1, 1]);
            d1pp = mkpp(obj.t, d1Coefs);
        end
        
        %------------------------------------------------------------------
        function d2pp = get.d2XPPoly(obj)
            n = length(obj.X);
            d2Coefs = obj.XPPoly.coefs(:, 1:2).* repmat([6 2], [n-1, 1]);
            d2pp = mkpp(obj.t, d2Coefs);
        end
        
        %------------------------------------------------------------------
        function d1pp = get.d1YPPoly(obj)
            n = length(obj.Y);
            d1Coefs = obj.YPPoly.coefs(:, 1:3).* repmat([3 2 1], [n-1, 1]);
            d1pp = mkpp(obj.t, d1Coefs);
        end
        
        %------------------------------------------------------------------
        function d2pp = get.d2YPPoly(obj)
            n = length(obj.Y);
            d2Coefs = obj.YPPoly.coefs(:, 1:2).* repmat([6 2], [n-1, 1]);
            d2pp = mkpp(obj.t, d2Coefs);
        end
    end
end   
