rows = 10;
cols = 5;
image_rows = 2000;
image_cols = 2000;
white = 1;
black = 0;
spacing = floor(image_rows/(rows+1));
s2 =   floor(2*spacing);
radius = spacing/2.25;
center1_x = 2*radius;
center1_y = 2*radius;
r_sq = radius*radius;

col_spacing = image_cols/(cols+1);
if col_spacing<spacing,
  spacing = col_spacing;
endif


for i=1:s2,
	for j=1:s2,
		dist2center_sq = (i-center1_x)*(i-center1_x) + (j-center1_y)*(j-center1_y);
                square1(i,j) = white;
                if dist2center_sq < r_sq,
                   square1(i,j) = black;
                endif
        end;
end;

mcircle(1000,1000) = 1;

for i=1:rows,
	for j=1:cols,
		for k=1:s2,
			for l=1:s2,
				ri = (i-1)*s2 + k;
                                ci = (j-1)*s2 + l;
				mcircle(ri,ci) = square1(k,l);
                        end;
                end;
        end;
end;

for i=1:s2,
	for j=1:s2,
		dist2center_sq = (i-center1_x)*(i-center1_x) + (j-center1_y)*(j-center1_y);
                square1(i,j) = white;
                if dist2center_sq < 2.5*r_sq,
                   mcircle(i,j) = black;
                endif
        end;
end;


imshow(mcircle);
print(figure(1),"mcircle.jpg","-djpg");
