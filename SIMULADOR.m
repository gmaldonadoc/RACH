% clear all;close all;clc;
% Nusers =10;
function [P_transmitido,P_sucess,P_colission,M_transmision, M_recepcion, detected, pdp, threshold,cyclicShift, p_env_unicos, p_uni_detected, FD, de_traunicos]= SIMULADOR(Nusers,factorregression)

Npreamble= 54;
prb=64; 
carrier = struct('NULRB',6,'DuplexMode','FDD', 'CyclicPrefixUL', 'Normal', 'NTxAnts', 1); %transportadora
foffset = 270.0;                        % Frequency offset in Hertz
prach = cell(1,Nusers);
radio=790;enbposition=[0,0];

    for u =1: Nusers    
        prach{u}.Format = 0; prach{u}.PreambleIdx =randi([20,22],1,1); prach{u}.SeqIdx = 22; prach{u}.CyclicShiftIdx = 1; prach{u}.HighSpeed = 0; prach{u}.FreqOffset = 0; prach{u}.Position=distuser(u,enbposition, radio);% estructura prach
        transmision{u}.EV= u;transmision{u}.UE= u;transmision{u}.P_Transmitido= prach{u}.PreambleIdx; transmision{u}.Posicion=prach{u}.Position;
        info = ltePRACHInfo(carrier, prach{u}); % info estructura prach
        chcfg=struct('NRxAnts',2,'DelayProfile','EPA','DopplerFreq',5.0 ,'InitTime',0,'MIMOCorrelation','low','Seed',1,'NTerms',16,'ModelType','GMEDS','InitPhase','Random','NormalizePathGains','On','NormalizeTxAnts','On','SamplingRate',info.SamplingRate); %parametros de channel
    end
   hPosition(prach,radio);
    % Generate transmit wave
    txwave = cell(1,Nusers);
    for u = 1:Nusers
        txwave{u}= ltePRACH(carrier,prach{u});
    end
    %  hPosition(prach,radio);
    %calcular distancia UE > eNB
    speedOfLight = 299792458.0;
    sampleDelay = zeros(1, Nusers);
    delays = []; 
    radius = cell(1, Nusers);
    for u = 1:Nusers
        [~, radius{u}] = cart2pol(prach{u}.Position(1), prach{u}.Position(2));
        transmision{u}.Distancia= radius{u};
        delay =((radius{u})/speedOfLight)*2;  
        sampleDelay(u) = ceil(delay*info.SamplingRate); 
    end


    if Nusers > 0
        for a = 1:length(transmision)
            ID_event(a)= transmision{a}.EV;
            ID_UE(a) = transmision{a}.UE;
            P_Enviado(a) = transmision{a}.P_Transmitido;
            Posicion_UE(a).fields = [prach{a}.Position(1), prach{a}.Position(2)];
            PosiX_UE(a) = prach{a}.Position(1);
            PosiY_UE(a) = prach{a}.Position(2);
            Dist_UE_eNB(a) = floor(transmision{a}.Distancia);
        end

        M_transmision = [ID_event' ID_UE' P_Enviado' PosiX_UE' PosiY_UE' Dist_UE_eNB'];

        [p_env_unicos]=P_unicos(P_Enviado);

        sum_rx = zeros(length(txwave{1}) + max(sampleDelay), 1);

        for u = 1:Nusers
            chcfg.SamplingRate = info.SamplingRate;
            [rx{u}, i] = lteFadingChannel(chcfg, ...
                [zeros(sampleDelay(u), 1); txwave{u}; ...
                zeros(max(sampleDelay) - sampleDelay(u), 1)]);
            sum_rx = sum_rx + rx{u};
        end

        % Remove the implementation delay of the channel modeling
        sumrx = sum_rx((i.ChannelFilterDelay + 1):end, :);

        % Apply frequency offset
        t = ((0:size(sumrx, 1) - 1) / info.SamplingRate).';
        sumrx = sumrx .* repmat(exp(1i * 2 * pi * foffset * t), ...
            1, size(sumrx, 2));

        % PRACH detection for preamble indices
        [detected, offsets, pdp, threshold, cyclicShift, picos, bestCorr] = PreambleDetection(factorregression, carrier, prach{u}, sumrx, (0:63).');

        [p_uni_detected]=P_unicos(detected); %info unique preambles 
        f = setdiff(detected,p_env_unicos); 
        de_traunicos= length(f); 
        [FD] = falsealarm(detected,P_Enviado);

        [datos] = TAC2(detected, offsets); % calcula time advance

        [alocados] = alocated(prb, datos); % asigna recursos aleatorios

        [Recepcion] = ACKNACK2(alocados, P_Enviado);

        for x = 1:length(Recepcion)
            N_event(x)= x;
            N_ceros(x) = x;
            P_recibido(x) = Recepcion{x}.Preambulo;
            TA_recibido(x) = Recepcion{x}.TA;
            Prb_recibido(x) = Recepcion{x}.Prb;
            M_Enb_UE(x) = Recepcion{x}.ACKNACK;
            M_recepcion = [N_event' N_ceros' P_recibido' TA_recibido' Prb_recibido' M_Enb_UE'];
        end

        [suce]= sucess (P_Enviado);
        P_sucess = length(suce)/Nusers;
        P_colission = (Nusers-length(suce))/Nusers;
        unique_preambles = unique(P_Enviado);
        P_transmitido= length(unique_preambles)/Npreamble;
    else
        M_transmision = []; % Set M_transmision to an empty matrix
        M_recepcion = [];   % Set M_recepcion to an empty array
        p_env_unicos = 0;
        P_sucess = 0;
        P_colission = 0;
        P_transmitido= 0;
        detected = [];
        pdp = [];
        threshold = 0;
        cyclicShift = [];
        p_uni_detected = 0;
        FD = 0;
        de_traunicos = 0;
    end
end


% figure
% stem(pdp,'b', 'LineWidth', 2);xlines(cyclicShift);yline(threshold,'--r','Threshold','LineWidth',1);hold on;title('Power profile delay'); legend('PDP','Bin','Threshold'); 
% plot(picos, bestCorr, 'ro', 'MarkerSize', 10, 'LineWidth',3); % Plot peaks with red circles
% % % % figure 
% % % % stem(pdp2,'b', 'LineWidth', 2);xlines(cyclicShift2);yline(threshold2,'--r','Threshold','LineWidth',1);hold on;title('Power profile delay 2'); legend('PDP','Bin','Threshold'); 
% % % % 
% % % % % Create the new matrix combining detected, detected2, and second_column
% % % % 
% % % % second = M_transmision(:, 2);
% % % % second_column = sort(second, 'descend');
% % % % % Calculate the maximum length among the vectors
% % % % max_length = max([length(detected), length(detected2), length(second_column)]);
% % % % 
% % % % % Pad the shorter vectors with 99
% % % % detected_padded = [detected, 99*ones(1, max_length - length(detected))];
% % % % detected2_padded = [detected2, 99*ones(1, max_length - length(detected2))];
% % % % second_column_padded = [second_column', 99*ones(1, max_length - length(second_column))];
% % % % 
% % % % % Create the new matrix combining padded vectors
% % % % new_matrix = [detected_padded', detected2_padded', second_column_padded'];
% % % % 
% % % % % Display the new matrix
% % % % disp(new_matrix);


