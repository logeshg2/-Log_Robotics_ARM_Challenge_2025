function color = detectcolor(img)
    % corp the image to cubes
    img = img(350:480, 200:450, :);
    
    % Convert to HSV
    hsvImg = rgb2hsv(img);
    
    hue = hsvImg(:,:,1);         
    saturation = hsvImg(:,:,2);  
    value = hsvImg(:,:,3);       

    % Define masks for each color
    redMask = (hue < 0.05 | hue > 0.95) & saturation > 0.5 & value > 0.3;
    blueMask = (hue > 0.55 & hue < 0.75) & saturation > 0.4 & value > 0.2;
    greenMask = (hue > 0.25 & hue < 0.4) & saturation > 0.4 & value > 0.2;
    purpleMask = (hue > 0.75 & hue < 0.9) & saturation > 0.4 & value > 0.2;
    
    % Clean masks
    se = strel('disk',5);
    redMask = imopen(redMask, se);
    blueMask = imopen(blueMask, se);
    greenMask = imopen(greenMask, se);
    purpleMask = imopen(purpleMask, se);
    
    % Check which colors are present
    color = "";
    
    if any(redMask(:))
        color = 'red';
    elseif any(blueMask(:))
        color = 'blue';
    elseif any(greenMask(:))
        color = 'green';
    elseif any(purpleMask(:))
        color = 'purple';
    end
end
