function centroid = segment_extrinsic(I, depth_frame)
    % initialise variables
    position = 0;
    %centroid = 0;
    centroid = zeros(2,1);
    obj_width = 0;
    obj_height = 0;
    
    Iycbcr = rgb2ycbcr(I);

    Icr = Iycbcr(:,:,3);% for ycbcr

    Ithres = Icr > 142; % for ycbcr (red chroma)

    SE = strel('disk', 14);

    Iopen = imopen(Ithres, SE);

    % Count connected objects
    [L, n] = bwlabel(Iopen);

    for i = 1:n
        Iopen = L == i;
                
        % Find the centroid of objects
        measurements = regionprops(Iopen, 'Centroid');
        centroids = [measurements.Centroid];
        xCentroids = centroids(1:2:end);
        yCentroids = centroids(2:2:end);
        
        centroid = centroids;
    
        % depth processing
        xt = round(xCentroids(1));
        yt = round(yCentroids(1));
        
        dist_array = 0;
         for k = xt-3 : xt
         for m = yt-3 : yt+3
               %distance = depth_frame.get_distance(k, m);
               distance = depth_frame(k, m);
                
                dist_array = [dist_array distance];
            end
        end  
                
        distance_avg = round(mean(nonzeros(dist_array)), 3);
        
        % Check if blob is within range, otherwise reject
        %if distance_avg > 0.55 || isnan(distance_avg)
            if distance_avg > 1 || isnan(distance_avg)
            continue;
        end
        
        position = distance_avg; 
        
        % check tomato size
        % compute object height
        majoraxis = regionprops(Iopen, 'MajorAxisLength');
        majoraxis_length = majoraxis.MajorAxisLength;
        
        obj_width = compute_height(majoraxis_length, distance_avg, 480, 42.5);
        obj_width = obj_width * 100; % convert to cm
                
        % compute object width
        minoraxis = regionprops(Iopen, 'MinorAxisLength');
        minoraxis_length = minoraxis.MinorAxisLength;
        
        obj_height = compute_height(minoraxis_length, distance_avg, 848, 69.4);
        obj_height = obj_height * 100; % convert to cm
        
        % Check if size is within range
        if obj_width < 5 && obj_width > 6 && ...
                obj_height < 4.5 && obj_height > 5.5
            continue;
        end  
        
        % Check circularity
        % For MATLAB 2018 and later
        perimeter_measure = regionprops(Iopen, 'Perimeter');
        perimeter = perimeter_measure.Perimeter;
        
        area_measure = regionprops(Iopen, 'Area');
        area = area_measure.Area;
        
        circularity = (4*area*pi)/(perimeter^2);
        
        % Only for MATLAB 2019
%             circularity_measure = regionprops(Iopen, 'Circularity');
%             circularity = circularity_measure.Circularity;

        % Check if circularity is within range, otherwise reject
        if circularity < 0.75 || circularity > 1
            continue;
        end
        
        % put boundary trace
        B = bwboundaries(Iopen, "noholes");
        for k = 1:length(B)
           boundary = B{k};
           plot(boundary(:,2), boundary(:,1), 'y', 'LineWidth', 2)
        end
        
        % put + on centre of mass
        text(xCentroids,yCentroids, "+",'Color','yellow','FontSize',14);
        
        % put distance next to object
        distance_str = strcat(mat2str(distance_avg), " meters");
        text(xCentroids + 10,yCentroids + 50, distance_str,'Color','yellow','FontSize',14);

    end
end

function obj_height = compute_height(obj_span, distance, frame_lw, fov)
    obj_height = obj_span * distance * 2 / frame_lw * tan(fov / 2 * pi / 180);
end
