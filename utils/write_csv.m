function write_csv(file, header, M)
%WRITE_CSV  Write a numeric matrix with a header row (works in MATLAB and Octave).
fid = fopen(file, 'w');
if fid < 0, error('write_csv: cannot open %s', file); end
fprintf(fid, '%s\n', strjoin(header, ','));
fmt = [repmat('%.10g,', 1, size(M,2)-1) '%.10g\n'];
fprintf(fid, fmt, M');
fclose(fid);
end
