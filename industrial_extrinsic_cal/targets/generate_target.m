function generate_target( file_name, rows, cols, circle_dia, spacing)
fp = fopen(file_name,"w");
fprintf( fp, "---\n\nstatic_targets:\n-\n");
fprintf(fp, "     target_name: modified_circle_%dx%d\n",rows,cols);
fprintf(fp, "     target_type: 2\n");
fprintf(fp, "     circle_dia: %7.4f\n", circle_dia);
fprintf(fp,"     target_frame: target_frame\n");
fprintf(fp,"     transform_interface: ros_lti\n");
fprintf(fp,"#      xyz_aaxis_pose: [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n");
fprintf(fp,"     target_rows: %d\n", rows);
fprintf(fp,"     target_cols: %d\n", cols);
fprintf(fp,"     num_points: %d\n", cols*rows);
fprintf(fp,"     points:\n");
for i=1:rows,
	y= (rows-i)*spacing;
                for j=0:(cols-1),
		x = j*spacing;
                                fprintf(fp,"      - pnt: [ %7.4f, %7.4f, 0.0000]\n", x, y );
end;
end;
fclose(fp);
end
