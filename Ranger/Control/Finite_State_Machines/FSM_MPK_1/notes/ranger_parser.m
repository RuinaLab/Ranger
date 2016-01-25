
 
fprintf('\n\n************************ RangerParser v1.0 ************************\n');
fprintf('           Emily McAdams, Nicolas Champagne-Williamson\n\n');
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Parsing
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf(': Parsing files... ');
 
%Parse board_table.csv
board_file = fopen('board_table.csv', 'r');
board_data = textscan(board_file, '%s%s%d%d%s', 'Delimiter', ',', 'CommentStyle', '#');
board_length = length(board_data{1});
board_ids = board_data{1};
board_nums = board_data{2};
board_can = board_data{4} + 2*board_data{3};
board_desc = board_data{5};
fclose(board_file);
 
%Parse can_table.csv
fid = fopen('can_table.csv','r');
can_data = textscan(fid,'%s%s%s%s%s','Delimiter',',', 'CommentStyle', '#');
can_length = length(can_data{1});
can_ids = can_data{1};
can_defaults = can_data{2};
can_srcs = can_data{3};
can_dests = can_data{4};
can_comments = can_data{5};
%replace board ids with numbers
can_srcs = regexprep(can_srcs, board_ids, board_nums);
can_dests = regexprep(can_dests, board_ids, board_nums);
can_split_dests = regexp(can_dests, ';', 'split');
fclose(fid);

%Parse error_table.csv
error_fid = fopen('error_table.csv','r');
error_data = textscan(error_fid, '%s%s%s%s','Delimiter',',','CommentStyle','#');
error_length = length(error_data{1});
error_module = error_data{1};
error_name = error_data{2};
error_priority = error_data{3};
error_desc = error_data{4};

fprintf('done!\n');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% board_id.h
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf(': Writing board_id.h... ');
 
file_0=fopen('board_id.h','w');
fprintf(file_0,'//\n');
fprintf(file_0,'// Board Names and Numbers for Cornell Ranger Robot\n');
fprintf(file_0,'//\n\n');
fprintf(file_0,'#ifndef __H_BOARD_ID__\n');
fprintf(file_0,'#define __H_BOARD_ID__ \n\n');
fprintf(file_0,'typedef enum boards{\n');
 
for i = 1:board_length
    if strcmp(board_nums(i),'')
        fprintf(file_0,'\tBOARD_%s,\t//\t%s\n', board_ids{i}, board_desc{i});
    else
        fprintf(file_0,'\tBOARD_%s = %s,\t//\t%s\n', board_ids{i}, board_nums{i}, board_desc{i});
    end
end
fprintf(file_0,'\tBOARD_LAST\n');
fprintf(file_0,'} BOARD_ID;\n');
fprintf(file_0,'\n\n#endif //__H_BOARD_ID__\n\n');
 
fclose(file_0);
fprintf('done!\n');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% error_id.h
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf(': Writing error_id.h... ');
 
error_fid=fopen('error_id.h','w');
fprintf(error_fid,'//\n');
fprintf(error_fid,'// Error Names and Comments for Cornell Ranger Robot\n');
fprintf(error_fid,'//\n\n');
fprintf(error_fid,'#ifndef __H_ERROR_ID__\n');
fprintf(error_fid,'#define __H_ERROR_ID__ \n\n');
fprintf(error_fid,'typedef enum errors{\n');
fprintf(error_fid,'\tERROR_DEFAULT,\n');
 
for i = 1:error_length
    fprintf(error_fid,'\tERROR_%s_%s,\t//\t%s: %s\n', error_module{i}, error_name{i}, error_module{i}, error_desc{i});
end

fprintf(error_fid,'\tERROR_LAST_ID\n');
fprintf(error_fid,'} ERROR_ID;\n');
fprintf(error_fid,'\n\n#endif //__H_ERROR_ID__\n\n');
 
fclose(error_fid);
fprintf('done!\n');
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% can_id.h
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf(': Writing can_id.h... ');
 
%Writing the CAN_ID.h file
file_1 = fopen('can_id.h','w');
fprintf(file_1,'//\n');
fprintf(file_1,'//CAN_ID Table for Cornell Ranger Robot\n');
fprintf(file_1,'//Automatically generated by file_read.m\n');
fprintf(file_1,'//\n\n');
fprintf(file_1,'#ifndef __H_CAN_ID__\n');
fprintf(file_1,'#define __H_CAN_ID__ \n\n');
fprintf(file_1,'typedef enum can_ids{\n');

can_num = 0;
for i=1:can_length
    if strcmp(can_ids{i}(1:2), 'LV')
        fprintf(file_1, '\tID_%s,\t //%s %s %s %s\n',can_ids{i}, can_defaults{i}, can_srcs{i}, can_dests{i}, can_comments{i});
    else
        fprintf(file_1, '\tID_%s = %d,\t //%s %s %s %s\n', can_ids{i}, can_num, can_defaults{i}, can_srcs{i}, can_dests{i}, can_comments{i});     
        can_num = can_num + 1;
    end
end

fprintf(file_1, '\tID_LAST\n');
fprintf(file_1,'} CAN_ID;\n\n\n');
fprintf(file_1,'#endif\n\n');
fclose(file_1);
 
fprintf('done!\n');
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% init_txlist.c & init_rxlist.c
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Create a C file -- init_txlist.c and init_rxlist.c
%txlist: Src = main brain, Dest = satellite
%rxlist: Src = satellite, Dest = main brain

fprintf(': Writing init_txlist.h and init_rxlist.h... ');
 
file_2 = fopen('init_txlist.h','w');
file_3 = fopen('init_rxlist.h','w');
 
fprintf(file_2, '//List of everything being transmitted from MB to satellites. \n');
fprintf(file_2, 'short unsigned int a9_dn_ssp_tx_list[ID_LAST + 2] = {\n');

fprintf(file_3, '//List of everything being transmitted from satellites to MB. \n');

for i = 1:can_length
    %add to txlist if src include MB (1) and dests includes satellite
    if (~isempty(regexp(can_srcs{i}, '[01]', 'once')) && ~isempty(regexp(can_dests{i}, '[345678]', 'once')))
        fprintf(file_2, '\tID_%s,\n', can_ids{i});
    end
    %add to rxlist if src include satellite (1) and dests includes MB
    if (~isempty(regexp(can_dests{i}, '[1]', 'once')) && ~isempty(regexp(can_srcs{i}, '[345678]', 'once')))
        fprintf(file_3, '\tID_%s,\n', can_ids{i});
    end
end

fprintf(file_2, '0xFFFF, 0xFFFF\n};\n\n');

fprintf('done!\n');
fclose(file_2);
fclose(file_3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% io_data.c
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Init defaults for ids with src = PC
fprintf(': Writing io_data.c... ');
 
file_4=fopen('io_data.c','w');
fprintf(file_4, '//Set initial values for every parameter (src is 0 - labview/PC)\n');
 
for i = 1:can_length
    if ~isempty(regexp(can_srcs{i}, '[0]', 'once'))
        float_val = str2double(can_defaults{i});
        fprintf(file_4, 'mb_io_data[ID_%s].payload.ful.f = %f; \n', can_ids{i}, float_val);
    end
end
 
fclose(file_4);
fprintf('done!\n');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% error_messages.h
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Create an array of string for the error message from MB to LCD
fprintf(': Writing error_messages.h... ');
 
emfid=fopen('error_messages.h','w');
fprintf(emfid, '//Error messages for MB to display on the LCD\n');
fprintf(emfid, 'char* mb_error_messages[%d] = {\n', error_length + 1);

for i = 1:error_length
    message = strcat(error_module{i}, '_', error_name{i});
    fprintf(emfid, '\t"%s",\n',message);
end

fprintf(emfid, '\t"LAST_ERROR"\n');
fprintf(emfid, '};\n\n');
fclose(emfid);
fprintf('done!\n');
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% csr_routing_table.c
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%Write the routing table used by the router board
fprintf(': Writing csr_routing_table.c... ');

rfid = fopen('csr_routing_table.c','w');
fprintf(rfid, '//CSR Packet Routing Table filled by ranger_parser.m\n\n');

for i = 1:can_length
    %src includes MB or PC and dests includes any of the satellites
    if (~isempty(regexp(can_srcs{i}, '[01]', 'once')) && ~isempty(regexp(can_dests{i}, '[345678]', 'once')))
        fprintf(rfid,'\tcsr_routing_table[ID_%s].destinations = ', can_ids{i});
        for j = 1:length(can_split_dests{i})-1
            can_channel = board_can(str2double(can_split_dests{i}{j})+1);
            fprintf(rfid,'(1<<CHAN_CAN%d)|', can_channel);
        end
        can_channel = board_can(str2double(can_split_dests{i}{length(can_split_dests{i})})+1);
        fprintf(rfid,'(1<<CHAN_CAN%d);\n', can_channel);
    end
end

fclose(rfid);
fprintf('done!\n');
            
    