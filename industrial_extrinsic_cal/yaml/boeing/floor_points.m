fp = fopen("floor_points.yaml","w")
fprintf(fp,"---\npoints:\n");
for x=0:15:300, for y=0:15:300, fprintf(fp,"- pnt: [ %f, %f, 0.0]\n",x,y); end; end;
fclose(fp);
