function img = make_track_image(W, H)
img = uint8(200 * ones(H, W, 3));
cx = W/2;  cy = H/2;
[xx, yy] = meshgrid(1:W, 1:H);
r  = sqrt((xx-cx).^2 + (yy-cy).^2);
ri = H*0.17;   ro = H*0.45;
m  = (r >= ri) & (r <= ro);
for c = 1:3, L=img(:,:,c); L(m)=155; img(:,:,c)=L; end
for bnd = [ri, ro]
    m2 = (r >= bnd-5) & (r <= bnd+5);
    for c = 1:3, L=img(:,:,c); L(m2)=255; img(:,:,c)=L; end
end
end