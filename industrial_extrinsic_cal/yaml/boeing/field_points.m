fp = fopen("field_points.yaml","w")
fprintf(fp,"---\npoints:\n");
for x=0:5:300, for y=0:5:260, fprintf(fp,"- pnt: [ %f, %f, 15.0]\n",x,y); end; end;
fclose(fp);
