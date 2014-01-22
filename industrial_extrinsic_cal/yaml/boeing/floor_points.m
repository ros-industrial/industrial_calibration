fp = fopen("floor_points.yaml","w")
fprintf(fp,"---\npoints:\n");
for x=0:5:300, for y=0:5:300, fprintf(fp,"- pnt: [ %f, %f, 0.0]\n",x,y); end; end;
fclose(fp);
