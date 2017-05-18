function result = getBeaconID(img)
    img = imgaussfilt(img, 5);
    hsv = rgb2hsv(img);
    
    gray = rgb2gray(img);
    [equ, T] = histeq(gray);
    [chrom, lum] = getChrom(img);
    
    m_black = lum < 70;
    m_black = m_black * 255;
    %%% Assuming that a transform to uint8 isn't required.
    
    %%% Color mask limits based on hsv values (gimp format: 0-360 0-100 0-100)
    lower_r1 = [0,120,50];         
    upper_r1 = [15*2.55,255,255]/255;     
    lower_r2 = [175*2.55,120,50]/255;        
    upper_r2 = [180*2.55,255,255]/255;     
    lower_g = [15*2.55,100,50]/255;         
    upper_g = [45*2.55,255,130]/255;        
    lower_b = [70*2.55,12,25]/255;        
    upper_b = [165*2.55,160,130]/255;        
    lower_fg = [45*2.55,112,25]/255;         
    upper_fg = [63*2.55,180,112]/255; 
    
    %%% Create color masks (note: blue mask inaccurate -> verify blue beacon segment by red and green -> green less precise than red (verify?))
    %m_r1 = inrange(hsv,lower_r1,upper_r1);
    m_r1 = (hsv(:,:,1) > lower_r1(1)) & (hsv(:,:,1) < upper_r1(1)) & (hsv(:,:,2) > lower_r1(2)) & (hsv(:,:,2) < upper_r1(2)) & (hsv(:,:,3) > lower_r1(3)) & (hsv(:,:,3) < upper_r1(3));
    m_r2 = (hsv(:,:,1) > lower_r2(1)) & (hsv(:,:,1) < upper_r2(1)) & (hsv(:,:,2) > lower_r2(2)) & (hsv(:,:,2) < upper_r2(2)) & (hsv(:,:,3) > lower_r2(3)) & (hsv(:,:,3) < upper_r2(3));
    m_r = m_r1 | m_r2;
    m_g = (hsv(:,:,1) > lower_g(1)) & (hsv(:,:,1) < upper_g(1)) & (hsv(:,:,2) > lower_g(2)) & (hsv(:,:,2) < upper_g(2)) & (hsv(:,:,3) > lower_g(3)) & (hsv(:,:,3) < upper_g(3));
    m_b = (hsv(:,:,1) > lower_b(1)) & (hsv(:,:,1) < upper_b(1)) & (hsv(:,:,2) > lower_b(2)) & (hsv(:,:,2) < upper_b(2)) & (hsv(:,:,3) > lower_b(3)) & (hsv(:,:,3) < upper_b(3));
    m_fg = (hsv(:,:,1) > lower_fg(1)) & (hsv(:,:,1) < upper_fg(1)) & (hsv(:,:,2) > lower_fg(2)) & (hsv(:,:,2) < upper_fg(2)) & (hsv(:,:,3) > lower_fg(3)) & (hsv(:,:,3) < upper_fg(3));
        
    %%% remove salt and pepper artifacts
    P = 10;
    filtered_m_r = bwareaopen(m_r,P);
    filtered_m_g = bwareaopen(m_g,P);
    filtered_m_b = bwareaopen(m_b,P);
    filtered_m_fg = bwareaopen(m_fg,P);
    
    result = m_r; 
end