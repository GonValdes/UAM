function ST_Data = Readfile_ST(basename)
%% Open the text file.
filename = [basename '.st'];
fileID = fopen(filename,'r');
text = fread(fileID, '*char').';
fclose(fileID); %file can be closed now

ST_Data = struct();

ST_Data.CL = readValueFromText(text,'CLtot');
% ST_Data.CD = readValueFromText(text,'CDtot');
ST_Data.Cm = readValueFromText(text,'Cmtot');
ST_Data.Alpha = readValueFromText(text,'Alpha');
ST_Data.Mach = readValueFromText(text,'Mach');

ST_Data.CDind = readValueFromText(text,'CDind');
ST_Data.CDff = readValueFromText(text,'CDff');

ST_Data.CLa = readValueFromText(text,'CLa');
ST_Data.CLb = readValueFromText(text,'CLb');
ST_Data.CYa = readValueFromText(text,'CYa');
ST_Data.CYb = readValueFromText(text,'CYb');
ST_Data.Cla = readValueFromText(text,'Cla');
ST_Data.Clb = readValueFromText(text,'Clb');
ST_Data.Cma = readValueFromText(text,'Cma');
ST_Data.Cmb = readValueFromText(text,'Cmb');
ST_Data.Cna = readValueFromText(text,'Cna');
ST_Data.Cnb = readValueFromText(text,'Cnb');

ST_Data.CLp = readValueFromText(text,'CLp');
ST_Data.CLq = readValueFromText(text,'CLq');
ST_Data.CLr = readValueFromText(text,'CLr');

ST_Data.CYp = readValueFromText(text,'CYp');
ST_Data.CYq = readValueFromText(text,'CYq');
ST_Data.CYr = readValueFromText(text,'CYr');

ST_Data.Clp = readValueFromText(text,'Clp');
ST_Data.Clq = readValueFromText(text,'Clq');
ST_Data.Clr = readValueFromText(text,'Clr');

ST_Data.Cmp = readValueFromText(text,'Cmp');
ST_Data.Cmq = readValueFromText(text,'Cmq');
ST_Data.Cmr = readValueFromText(text,'Cmr');

ST_Data.Cnp = readValueFromText(text,'Cnp');
ST_Data.Cnq = readValueFromText(text,'Cnq');
ST_Data.Cnr = readValueFromText(text,'Cnr');

%inner_aileron
ST_Data.CLdinA = readValueFromText(text,'CLd2');
ST_Data.CmdinA = readValueFromText(text,'Cmd2');
ST_Data.CDdinA = readValueFromText(text,'CDffd2');
%outer_aileron
ST_Data.CldoutA = readValueFromText(text,'Cld1');
ST_Data.CYdoutA = readValueFromText(text,'CYd1');
ST_Data.CndoutA = readValueFromText(text,'Cnd1');
ST_Data.CDdoutA = readValueFromText(text,'CDffd1');
%Right rudder
ST_Data.CldRr = readValueFromText(text,'Cld3');
ST_Data.CYdRr = readValueFromText(text,'CYd3');
ST_Data.CndRr = readValueFromText(text,'Cnd3');
ST_Data.CDdRr = readValueFromText(text,'CDffd3');
% %Left rudder
% ST_Data.CldRl = readValueFromText(text,'Cld4');
% ST_Data.CYdRl = readValueFromText(text,'CYd4');
% ST_Data.CndRl = readValueFromText(text,'Cnd4');
% ST_Data.CDdRl = readValueFromText(text,'CDffd4');



% ST_Data.Np =readValueFromText(text,'Xnp');
% ST_Data.Spiral = readValueFromText(text,'Clb Cnr / Clr Cnb');

end

function value = readValueFromText(text,fieldName)
    %% Read a value from a AVL text stream using Regular expresions (REGEXP)
    regex = strcat(fieldName,'\s*=\s*(-?\d*[.]?\d+)');
    field_text =  regexp(char(text), regex,'tokens');
    value = str2num(char(field_text{1}));
end