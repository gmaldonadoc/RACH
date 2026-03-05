%%Crear archivo de probabilidades
matM=readtable('M_transmision_R790_Nant2.csv');
matR=readtable('M_recepcion_R790_Nant2.csv');
resulta =innerjoin(matM,matR,'LeftKeys',["Evento" "RASlot" "Preamble"],'RightKeys',["Evento" "RASlot" "Preamble"],"RightVariables","TA");
resulta.X_axis=[];
resulta.Y_axis=[];
writetable(resulta,'MATRIXTA_R790_Nant2_ETU70.csv');
%%crear archivo de prob
nnnnn= 1:37520;
nnnnn = nnnnn';
P_detectXrod = P_detectXrod';
P_FAXrod = P_FAXrod';
Preambletra = Preambletra';
Psucess = Psucess';
Pcolission = Pcolission';
mtotal= mean(FD);
MP = array2table(FD);
MP.Properties.VariableNames(1:6) = {'RASlot','Pdetectxrod','PfaXrod','Preambletra','Psucess','Pcolission'};
% mm=table2array(matM);
writetable(MP,'MATRIXPROB_R790_Nant2_ETU70.csv');


%%criar archivo dataset
matrix1 = readtable('PDP_Sig_R790_Nant2.csv');
matrix2 = readtable('M_recepcion_R790_Nant2.csv');
% Realizar un outer join
matrixresult = outerjoin(matrix1, matrix2, 'LeftKeys', ["Evento", "RASlot", "Preamble"], 'RightKeys', ["Evento", "RASlot", "Preamble"], 'RightVariables', "NACK_ACK", 'Type', 'left', 'MergeKeys', true);
% Reemplazar los valores NaN en la columna NACK_ACK con 3
matrixresult.NACK_ACK(isnan(matrixresult.NACK_ACK)) = 2;
% Guardar el resultado en un nuevo archivo CSV si es necesario
writetable(matrixresult, 'dataset_matlab_label3.csv');
% Perform an outer join
matrixresult = outerjoin(matrix1, matrix2, 'LeftKeys', ["Evento" "RASlot" "Preamble"], 'RightKeys', ["Evento" "RASlot" "Preamble"], 'RightVariables', "NACK_ACK");
% Replace NaN values with 0 in the NACK_ACK column
matrixresult.NACK_ACK(isnan(matrixresult.NACK_ACK)) = 0;
writetable(matrixresult,'DS_MUL_30mi_User_10even_R790_Nant2_EPA5.csv');

% % % % % % % % % matrix2= readtable('M_transmisionc.csv');
% % % % % % % % % roundn = @(x,n) round(x*10^n)./10^n;
% % % % % % % % % matrix2=table2array(matrix2);
% % % % % % % % % q = roundn(matrix2,2);
% matrix1=table2array(matrix1);
% matrix2=table2array(matrix2);
% result = outerjoin(matrix1,matrix2,"Type","left","LeftKeys",["Rodada" "Preamble"],"RightKeys",["Rodada" "Preamble"],"RightVariables","NACK_ACK");
% writetable(matrix1,'datasetPDP.csv');
% result = outerjoin(matrix1,matrix2,"left","LeftKeys",["Rodada" "Preamble"],"RightKeys",["Rodada" "Preamble"],"RightVariables","NACK_ACK");

PROVA=readtable('DS_B34_30mi_User_10even_R790_Nant2.csv');
provasort= sortrows(PROVA,28);
writetable(provasort,'DATAset_120.csv');
% extraerx = matM (1:30000,4);
% extraerx = table2array (extraerx);
% extraery = matM (1:30000,5);
% extraery = table2array (extraery);
% plot (extraerx,extraery,'r*')