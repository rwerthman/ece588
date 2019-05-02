function [centerFound, midpoint] = findCenter(cameraImage, velocity_pub)
    trim=cameraImage(130:210,40:end,:);
    hsv=rgb2hsv(trim);
    hsv_sat=hsv(:,:,2);
    
%     figure; imshow(trim);
%     figure; imshow(hsv_sat);
    
    %Use a canny edge detection method here.
    bwth=imbinarize(hsv_sat, 0.6);
    [bw_Canny,~] = edge(bwth,'Canny', 3*[1.2*0.0250, 0.0625]);
    
    [H,T,R] = hough(bw_Canny,'RhoResolution',0.5,'ThetaResolution',0.5);
    numpeaks = 1; %Specify the number of peaks
    P  = houghpeaks(H,numpeaks);
    lines = houghlines(bw_Canny,T,R,P,'FillGap', 50,'MinLength', 20); 
    
    if ~isempty(lines)
        stopTurtleBot(velocity_pub);
        max_len = 0;
        for k = 1:length(lines)
            xy = [lines(k).point1; lines(k).point2];
           
            % Determine the endpoints of the longest line segment
            len = norm(lines(k).point1 - lines(k).point2);
            if ( len > max_len)
               max_len = len;
               xy_long = xy;
               % Find the midpoint of the line which is the middle of the
               % line in camera
               midpoint = [((xy_long(1,1) + xy_long(2,1))/2), ((xy_long(1,2) + xy_long(2,2))/2)];
            end
        end
        figure; imshow(cameraImage);
        figure; imshow(trim); hold on;
        plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
        plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
        plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
        plot(midpoint(1),midpoint(2),'x','LineWidth',2,'Color','blue');
        midpoint = [midpoint(1), midpoint(2) +40];
        centerFound = true;
    else
        centerFound = false;
        midpoint = [0,0];
    end   
end