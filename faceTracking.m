%% Face Detection and Tracking Using the KLT Algorithm
% This example shows how to automatically detect and track a face using
% feature points. The approach in this example keeps track of the face even
% when the person tilts his or her head, or moves toward or away from the
% camera.
%
%   Copyright 2013 The MathWorks, Inc.
%% close all
%% Dimitri LECA
%% Maher NADAR
%%

%% We clean our workspace.
close all
clear all
clc

%% Choosing the method
% method = 1 -> using visiblePoints as the oldpoints
% % head is only tilting face and it referes to the initial frame so loss
% of points may occur.

% method = 2 -> Using all the points as the oldpoints
%% all features of first frame are kept andthere will be wrong tracking 


% method = 3 -> Using all the points INSIDE the bounding box as the
%% best 1 so far: we use vsible points to create the bbox and then save the reference features as the combination of the visible points and the features detected in the current frame
%% drawbacks: if faces intersect or a hand passes through a face, the tracking is lost.

% method = 4 -> Always comparing the markers with the ones found in the
% FIRST frame.
% oldpoints
method = 3;


% Display the detected points.
markerInserter = vision.MarkerInserter('Shape', 'Plus', 'BorderColor', 'White');       
markerInserterRed = vision.MarkerInserter('Shape', 'Plus', 'BorderColor', 'Custom', 'CustomBorderColor', uint8([255 0 0]));       
markerInserterBlue = vision.MarkerInserter('Shape', 'Plus', 'BorderColor', 'Custom', 'CustomBorderColor', uint8([0 0 255]));    
textInserter = vision.TextInserter('%d', 'LocationSource', 'Input port', 'Color',  [255, 255, 0], 'FontSize', 24);
shapeInserter = vision.ShapeInserter('Shape', 'Polygons', 'BorderColor','Custom', 'CustomBorderColor', [255 255 0]);



%% Detect a Face
% Create a cascade detector object.
faceDetector = vision.CascadeObjectDetector();
% Open a video file
videoFileReader = vision.VideoFileReader('obama.mp4');

% Read a video frame and run the face detector.

for i = 1:5 % We can select the first frame of our video.
    videoFrame      = step(videoFileReader);
end

% We save the first frame;
firstFrame = videoFrame;
% We call the faceDetector.
bbox = step(faceDetector, videoFrame);

% At this point, we are still initalizing our detector. 


% Convert the box to a polygon. This is needed to be able to visualize the
% rotation of the object.

% The number of people is the number of bbox returned by the faceDetector
% function.

numberPeople = size(bbox,1);

% We create some useful "cell". Here there will be used as "array of
% objects".
cell_PT = cell(numberPeople,1);
cell_points = cell(numberPeople,1);
cell_old_points = cell(numberPeople,1);
cell_Xface = cell(numberPeople,1);
cell_nb_markers = cell(numberPeople,1);

for n = 1:numberPeople % For each face detected.
    
    % We convert the initial bounding box (which are given using the
    % top-left corner and the width and height.
    
    bboxn = bbox(n,:);
    x = bboxn(1); y = bboxn(2); w = bboxn(3); h = bboxn(4);
    bboxPolygon(n,:) = [x, y, x+w, y, x+w, y+h, x, y+h];
    
    cell_Xface{n}(1,:) = [x + w/2 , y + h/2]; 

    % Draw the returned bounding box around the detected face.
    videoFrame = step(shapeInserter, videoFrame, bboxPolygon);
    
    figure; 
    imshow(videoFrame); 
    title('Detected face');

    %% Identify Facial Features in the initial frame To Track
    % Crop out the region of the image containing the face, and detect the
    % feature points inside it.
    %
    % There are three methods available in MATLAB
    %
    cornerDetector = vision.CornerDetector('Method', 'Minimum eigenvalue (Shi & Tomasi)');
    % cornerDetector = vision.CornerDetector('Method', 'Local intensity comparison (Rosten & Drummond)');
    % cornerDetector = vision.CornerDetector('Method', 'Harris corner detection (Harris & Stephens)');
    
    % We crop our image using imcrop, around the bbox, and we run our
    % corner detection.
    points = step(cornerDetector, rgb2gray(imcrop(videoFrame, bboxn)));     
    
    cell_nb_markers{n} = size(points,1);

    % The coordinates of the feature points are with respect to the cropped 
    % region. They need to be translated back into the original image
    % coordinate system. 
    points = double(points);
    points(:, 1) = points(:, 1) + double(bboxn(1));
    points(:, 2) = points(:, 2) + double(bboxn(2));

    

    

    videoFrame = step(markerInserter, videoFrame, points);
    
    figure;
    imshow(videoFrame);
    title('Detected features');

    %% Initialize a Tracker to Track the Points
    % Create a point tracker and enable the bidirectional error constraint to
    % make it more robust in the presence of noise and clutter.
    cell_PT{n} = vision.PointTracker('MaxBidirectionalError', 2);

    % Initialize the tracker with the initial point locations and the initial
    % video frame.
    initialize(cell_PT{n}, double(points), rgb2gray(videoFrame));
    cell_points{n} = points;
    
end

%% Initialize a Video Player to Display the Results
% Create a video player object for displaying video frames.
videoInfo = info(videoFileReader);
videoPlayer = vision.VideoPlayer('Position', [100 100 videoInfo.VideoSize(1:2)+30]);

%% Initialize a Geometric Transform Estimator
geometricTransformEstimator = vision.GeometricTransformEstimator('PixelDistanceThreshold', 4, 'Transform', 'Nonreflective similarity');

% Make a copy of the points to be used for computing the geometric
% transformation between the points in the previous and the current frames
for n = 1:numberPeople
    cell_oldPoints{n} = double(cell_points{n});
end

%% Track the Points
while ~isDone(videoFileReader) % While the video is running.
    
    % get the next frame
    videoFrame = step(videoFileReader);
    
    for n = 1:numberPeople % For each people detected.

        % Track the points. Note that some points may be lost.
        [points, isFound] = step(cell_PT{n}, rgb2gray(videoFrame));
        visiblePoints = points(isFound, :);
        oldInliers = cell_oldPoints{n}(isFound, :);
        bboxPolygon_n = bboxPolygon(n,:);
    
        if (size(visiblePoints,1) > 3) % We need at least three matched makers to do our analysis.
            
            % Estimate the geometric transformation between the old points
            % and the new points.
            [xform, geometricInlierIdx] = step(geometricTransformEstimator, double(oldInliers), double(visiblePoints));
        
            % Eliminate outliers
            visiblePoints = visiblePoints(geometricInlierIdx, :);
            oldInliers = oldInliers(geometricInlierIdx, :);
        
            % Apply the transformation to the bounding box
            boxPoints = [reshape(bboxPolygon_n, 2, 4)', ones(4, 1)];
            boxPoints2 = boxPoints;
            boxPoints = boxPoints * xform;
            bboxPolygon_n = reshape(boxPoints', 1, numel(boxPoints));
            bboxPolygon(n,:) = bboxPolygon_n;
            
            cell_Xface{n} = cat(1, cell_Xface{n},[(bboxPolygon_n(1)+bboxPolygon_n(5))/2, (bboxPolygon_n(2)+bboxPolygon_n(6))/2]);
                    
            % Insert a bounding box around the object being tracked
            videoFrame = step(shapeInserter, videoFrame, bboxPolygon_n);
            videoFrame = step(textInserter, videoFrame, int32(n), int32([bboxPolygon_n(1) bboxPolygon_n(2)]));

        
           

            % Display tracked points
            
            % We display at first all the markers found in the current
            % frame, inside de bounding box.
            %videoFrame = step(markerInserterRed, videoFrame, points);
            %videoFrame = step(markerInserter, videoFrame, pointsInsideBox);
            videoFrame = step(markerInserterBlue, videoFrame, visiblePoints);


            % Using another color, we display then the points "tracked"
            % from the previous frame.
            % videoFrame = step(markerInserterRed, videoFrame, visiblePoints);


            % Reset the points
            
            if (method == 1)
                oldPoints = visiblePoints;
            elseif(method == 2)
                oldPoints = points;
            elseif(method == 4)
                oldPoints = double(cell_points{n});
            else
                 % In the current frame we want to keep all the markers inside
                 % the new bounding box in order to use them in the following
                 % frame analysis. Then we have to "remove" points which are
                 % outside de bounding box.
                pointsInsideBox =  isInRect([bboxPolygon_n(1),bboxPolygon_n(2)], [bboxPolygon_n(3),bboxPolygon_n(4)], [bboxPolygon_n(5),bboxPolygon_n(6 )], [bboxPolygon_n(7),bboxPolygon_n(8)], points);
                oldPoints = pointsInsideBox;
            end
            
            cell_nb_markers{n} = cat(1,cell_nb_markers{n}, size(oldPoints,1));    
            setPoints(cell_PT{n}, abs(oldPoints));  
            cell_oldPoints{n} = oldPoints;
        end
    
    end
    % Display the annotated video frame using the video player object
    step(videoPlayer, videoFrame);

end

%% Clean up
release(videoFileReader);
release(videoPlayer);
release(geometricTransformEstimator);
release(cell_PT{:});
