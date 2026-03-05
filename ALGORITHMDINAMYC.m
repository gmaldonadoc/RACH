% function xx = ALGORITHMDINAMYC (Nevent ,NUEcell, Nslot)
%%variables 

clc;clearvars;close all;
Npreamble=54;   
radio= 790;
NRxAnts= 2;
RAOS= 2100; % Nº de RAO////////6 
NUEcell=30000;
Nevent=10; % numero de eventos
Nslot= RAOS;
%% almacenar datos
Preambletra=[];
Psucess =[];
Pcolission=[];
vec_falsealarm=[];
vec_falsealarm2=[];
vec_enviadoscount=0;
vec_detectedcount=0;
P_detectXrod =[];
dataPDP = zeros(64,27);

global TotalUePerRaSlot
[TotalUePerRaSlot,Mi] = TotalUeArrivals (NUEcell,Npreamble,RAOS);

for x = 1: Nevent  % for principal
    Nueslot = 0;
    
    for i = 1: Nslot % for anidado

        Nueslot = Nueslot + getNueslot(i); % genero usuarios en un determinado slot siguiendo la curva 

        if Nueslot >= 1 
            Nuenow = floor(Nueslot);
            Nueslot = Nueslot-Nuenow;  
        else
            Nuenow = 0;
            %Se no se transmite ningun usuario
        end
        Nuenow =5;
        factorregression = getfactor(Nuenow);
        %scenario already implemented
        
        [P_transmitido, P_sucess, P_colission, M_transmision, M_recepcion, detected, pdp, th, bins, enviadoscount, detectedcount, FD, de_traunicos]= SIMULADOR (Nuenow, factorregression);
        
        if ~isempty(pdp)
            % Only execute the following code if the variables are not empty
            %% guardar datos de probabilidades
            Preambletra = [Preambletra P_transmitido];
            Psucess = [Psucess P_sucess];
            Pcolission = [Pcolission P_colission];
            %datos para esa intancia
            %% guardar datos de probabilidades
            Preambletra= [Preambletra P_transmitido];
            Psucess= [Psucess P_sucess];
            Pcolission=[Pcolission P_colission];
            %% Detección de falsas alarmas
            vec= FD/enviadoscount;
            vec_falsealarm =[vec_falsealarm vec];
            vec2= FD/(64-enviadoscount);
            vec_falsealarm2 = [vec_falsealarm2 vec2];
            %% Detección probabilidad
            vec_enviadoscount=vec_enviadoscount+enviadoscount;
            vec_detectedcount= vec_detectedcount+detectedcount;
            b= vec_detectedcount/vec_enviadoscount;
            P_detectXrod= [P_detectXrod b];
            %%crear datasetPDP
%             c=round(pdp, 6);
            c= (pdp).';
            c=reshape(c,24,[]).';
            pattern = i; %numero de rodada
            patternevent = x; %numero de evento
            pream = [0; (63:-1:1).'];
            patternpreamevent = repmat(patternevent, ceil(size(pream,1) / length(patternevent)), 1);%numero de rodada por cantidad de preambulos
            patternpream = repmat(pattern, ceil(size(pream,1) / length(pattern)), 1);%numero de rodada por cantidad de preambulos

            dataPDP(:,1) = patternpreamevent(1:size(dataPDP,1));%insertar N_rodada 
            dataPDP(:,2) = patternpream(1:size(dataPDP,1));%insertar N_rodada 
            dataPDP(:,3) = pream(1:size(dataPDP,1));%insertar preambulos 
            dataPDP(:,4) = c(1:size(dataPDP,1));%insertar partes de señal
            xyz=5;        
            for gmc =2:24
                dataPDP(:,xyz) = c(:, gmc);%insertar N_rodada 
                xyz=xyz+1;
            end
            %% crear archivos .csv
            % Crear matrix PDP puro.
            matrixpdp = sortrows(dataPDP,2);
            P = array2table(matrixpdp);
            P.Properties.VariableNames(1:27) = {'Evento','RASlot','Preamble','PDP_S1','PDP_S2','PDP_S3','PDP_S4','PDP_S5','PDP_S6','PDP_S7','PDP_S8','PDP_S9','PDP_S10','PDP_S11','PDP_S12','PDP_S13','PDP_S14','PDP_S15','PDP_S16','PDP_S17','PDP_S18','PDP_S19','PDP_S20','PDP_S21','PDP_S22','PDP_S23','PDP_S24'};
            radio= num2str(radio);
            NRxAnts = num2str(NRxAnts);
            writetable(P,['PDP_Sig' '_R' num2str(radio,'%.2f') '_Nant' num2str(NRxAnts,'%.2f') '.csv'],'WriteMode','Append');
                  %         end
            %%Crear matrix M_infogeral  
            M_infogeral(:,1) = x;%insertar N_evento 
            M_infogeral(:,2) = i;%insertar RAOs 
            M_infogeral(:,3) = Nuenow;%insertar usuarios
            G = array2table(M_infogeral);
            G.Properties.VariableNames(1:3) = {'Evento','RASlot','NUEs'};
            writetable(G,['Device_number_per_rao','.csv'],'WriteMode','Append');

    %         if Nuenow == 0
            if isempty (M_transmision)
            % Crear matrix M_transmision 
                M_transmision = [];
            else
                M_transmision=round(M_transmision, 2);
                patternTransevent = repmat(patternevent, ceil(size(M_transmision,1) / length(patternevent)), 1);%numero de rodada de evento
                patternTrans = repmat(pattern, ceil(size(M_transmision,1) / length(pattern)), 1);%numero de rodada por cantidad de preambulos
                M_transmision(:,1) = patternTransevent(1:size(M_transmision,1));%insertar N_evento 
                M_transmision(:,2) = patternTrans(1:size(M_transmision,1));%insertar N_rodada 
                M = array2table(M_transmision);
                M.Properties.VariableNames(1:6) = {'Evento','RASlot','Preamble','X_axis','Y_axis','Dist'};
                writetable(M,['M_transmision' '_R' num2str(radio,'%.2f') '_Nant' num2str(NRxAnts,'%.2f') '.csv'],'WriteMode','Append');
            end
            % Crear matrix M_recepcion
            if ~ isempty (M_recepcion)

                matrixrecep = sortrows(M_recepcion,2);
                patternRecepevent = repmat(patternevent, ceil(size(matrixrecep,1) / length(patternevent)), 1);%numero de rodada por cantidad de preambulos
                patternRecep = repmat(pattern, ceil(size(matrixrecep,1) / length(pattern)), 1);%numero de rodada por cantidad de preambulos
                matrixrecep(:,1) = patternRecepevent(1:size(matrixrecep,1));%insertar N_rodada 
                matrixrecep(:,2) = patternRecep(1:size(matrixrecep,1));%insertar N_rodada 
                R = array2table(matrixrecep);
                R.Properties.VariableNames(1:6) = {'Evento','RASlot','Preamble','TA','PRB','NACK/ACK'};
                writetable(R,['M_recepcion' '_R' num2str(radio,'%.2f') '_Nant' num2str(NRxAnts,'%.2f') '.csv'],'WriteMode','Append');
            end
        end
    end
end

sum=0;

for t=1:length (vec_falsealarm)
    sum = sum + vec_falsealarm(t);
end

probabilidadFA=sum /length (vec_falsealarm);
resul=probabilidadFA*100;
probabilidadDE= vec_detectedcount/vec_enviadoscount;

%% guardar datos
P_FAXrod =[]; 
for lm=1:length(vec_falsealarm)
    a=vec_falsealarm(lm)/length(vec_falsealarm);
    P_FAXrod =  [P_FAXrod a];
end
% interpolación linear 
function factor = getfactor(Nuser)
    if Nuser == 0
        factor = 300;
    elseif Nuser == 1
        factor = 166;
    elseif Nuser <= 10
        factor = 166 - (Nuser - 1) * (166 - 64.5) / 9;
    elseif Nuser <= 40
        factor = 64.5 - (Nuser - 10) * (64.5 - 33.5) / 30;
    elseif Nuser <= 80
        factor = 33.5 - (Nuser - 40) * (33.5 - 16.6) / 40;
    elseif Nuser <= 120
        factor = 16.6 - (Nuser - 80) * (16.6 - 12.7) / 40;
    elseif Nuser <= 320
        factor = 12.7 - (Nuser - 120) * (12.7 - 10.2) / 200;
    end
end


function Nueslot = getNueslot(i)
    global TotalUePerRaSlot;
    Nueslot = TotalUePerRaSlot(i); 
end

% %%local functions
% function factor = getfactor (Nuser)
%     if Nuser == 0
%         factor = 300;
%     else
% %         factor= 209.79*(NUEcell^(-0.63));
%         factor = 166*(Nuser^(-0.63));
%     end
% end