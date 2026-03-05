
function [indout,offset,pdp,threshold,cyclicShift,picos,bestCorr] = PreambleDetection(factorregression,ue,config,waveform,indin)
           
    % Pre-configure empty outputs.
    indout=[];
    offset=[];
    
    % Validate the preamble indices length
    if (numel(indin)< 1 || numel(indin)>64)
        error('lte:error','INDIN must be between 1 and 64 in length, containing values between 0 and 63');
    end
    
    % Validate the preamble indices
    if (any(indin < 0) || any (indin>63))
        error('lte:error','INDIN must contain values between 0 and 63');
    end
    
    % Remove any timing offset in the input configuration.
    config.TimingOffset=0.0;
    
    % Remove ConfigIdx field in the input configuration so as not to limit
    % the search to any particular PRACH Configuration Index that might
    % be present in the input configuration. 
    if (isfield(config,'Format'))
        if (isfield(config,'ConfigIdx'))
            config=rmfield(config,'ConfigIdx');
        end
    end
    
    % Set up default value for DuplexMode field if it is absent.
    if (~isfield(ue,'DuplexMode'))
        ue.DuplexMode='FDD';
        lte.internal.defaultValueWarning('DuplexMode','FDD');
    end
            
    % If Format field is absent, infer it from the ConfigIdx field.
    config.Format = lte.internal.getPRACHFormat(ue,config);    
    
    % Set up default value for TDDConfig, SSC and FreqIdx fields if they are absent.
    if (strcmpi(ue.DuplexMode,'TDD'))
        if (~isfield(ue,'TDDConfig'))
            ue.TDDConfig = 0;
            lte.internal.defaultValueWarning('TDDConfig','0');
        end
        if (~isfield(ue,'SSC'))
            ue.SSC = 0;
            lte.internal.defaultValueWarning('SSC','0');
        end
        if (~isfield(config,'FreqIdx'))
            config.FreqIdx = 0;
            lte.internal.defaultValueWarning('FreqIdx','0');
        end
    end
    
    % Set up default value for HighSpeed field if it is absent.
    if (~isfield(config,'HighSpeed'))
        config.HighSpeed=0;
        lte.internal.defaultValueWarning('HighSpeed','0');
    end
    
    % Set up default value for SeqIdx field if it is absent.
    if (~isfield(config,'SeqIdx'))
        config.SeqIdx=0;
        lte.internal.defaultValueWarning('SeqIdx','0');
    end
    
    % Set up default value for CyclicShiftIdx field if it is absent.
    if (~isfield(config,'CyclicShiftIdx'))
        config.CyclicShiftIdx=0;
        lte.internal.defaultValueWarning('CyclicShiftIdx','0');
    end
    
    % Set up default value for NSubframe and NFrame fields if they are absent.
    if (isfield(config,'ConfigIdx') || strcmpi(ue.DuplexMode,'TDD'))
        if (~isfield(ue,'NSubframe'))
            ue.NSubframe = 0;
            lte.internal.defaultValueWarning('NSubframe','0');
        end
        if (~isfield(ue,'NFrame'))
            ue.NFrame = 0;
            lte.internal.defaultValueWarning('NFrame','0');
        end
    end
    
    % Set up default value for FreqOffset and CyclicPrefix fields if they are absent.
    if (config.Format~=4)
        if (~isfield(config,'FreqOffset'))
            config.FreqOffset = 0;
            lte.internal.defaultValueWarning('FreqOffset','0');
        end
    else
        if (~isfield(ue,'CyclicPrefix'))
            ue.CyclicPrefix = 'Normal';
            lte.internal.defaultValueWarning('CyclicPrefix','Normal');
        end
    end
    
    % Fundamental sampling rate.
    Fs=30.72e6;    
        
    % find set of root sequences required for set of input preamble indices.
    config.PreambleIdx=0:63;
    info=ltePRACHInfo(ue,config);    
    u=unique(info.RootSeq(indin+1));      
             
    % Set up of number of receive antennas 'NRxAnts', correlation outputs 'c'
    % first preamble index for each distinct root sequence 'preambleidx' and
    % detection threshold for each antenna 'threshold'.
    NRxAnts=size(waveform,2);    
    c=cell(length(u),1);
    preambleidx=zeros(length(u),1);
    threshold=zeros(NRxAnts,1);
    
    % Configure correlator input dimensions
    start=(info.Fields(1)+info.Fields(2))/Fs*info.SamplingRate;
    duration=info.Fields(3)/Fs*info.SamplingRate;
    
    % For format 2 and 3, perform 2 correlations, one on each half of
    % the useful PRACH symbol. For other formats, perform one correlation.
    if (config.Format==2 || config.Format==3)
        duration=duration/2;
        ncorrs=2;
    else
        ncorrs=1;
    end
    
    % Perform all necessary correlations i.e. those with distinct root sequences.
    for i=1:length(u)

        % Find first preamble index preambleidx(i) to give root sequence u(i).
        preambleidx(i)=find(info.RootSeq==u(i),1)-1;

        % Create reference PRACH sequence.
        config.PreambleIdx=preambleidx(i);        
        refPRACH=ltePRACH(ue,config);
        refPRACH=refPRACH(start+(1:duration));
        N=0.156992901969349;
        noise = N*complex(randn(size(waveform)), randn(size(waveform))); 
        waveform = waveform + noise;
        % for each receive antenna
        for p=1:NRxAnts
            
            % Perform correlation(s) of input on the pth antenna with reference 
            % and store result.
            cp=zeros(duration,1);
            
            for k=1:ncorrs
                rx=waveform(start+((k-1)*duration)+(1:duration),p)/Fs*info.SamplingRate;
                rxFFT=fft(rx);                        
                cp=cp+abs(ifft(rxFFT.*conj(fft(refPRACH)))).^2;
            end
            cp=cp/sqrt(ncorrs);
                       
            % Form threshold for subsequent detection based on estimation of
            % the received signal power on pth antenna within PRACH bandwidth.
            % The ratio info.K/12 deals with difference between expected 
            % level of correlation output between formats 0-3 and format 4.
            % The scaling factor of 166 has been determined empirically to 
            % meet target false alarm rate in AWGN. 
            if (i==1)
                mask = abs(fft(refPRACH))>0.1;
                threshold(p) = var(ifft(rxFFT.*mask)) * factorregression * (info.K/12);                
            end
            
            % Combine correlations from each antenna.
            if (p==1)
                c{i}=cp;
            else
                c{i}=c{i}+cp;
            end            
            
        end
        c{i}=c{i}/NRxAnts;
        pdp = c{i};
        % For High Speed mode, determine the size of the cyclic offset due 
        % to Doppler shift, in order to deal with the side peaks caused by 
        % loss of orthogonality due to high Doppler shift. The side peaks
        % are dealt with here by combining each interval of length 
        % 'cyclicOffset' in the correlation output. 
        if (config.HighSpeed)        
            cyclicOffset=fix((info.CyclicOffset(preambleidx(i)+1)/info.NZC)*info.SamplingRate/info.SubcarrierSpacing);                                            
            x=c{i};            
            c{i}=x((1+cyclicOffset):end)+x(1:(end-cyclicOffset))/sqrt(2);
            if (info.CyclicShift(preambleidx(i)+1)==-1)                
                c{i}=zeros(size(c{i}));
            end
        end        
        
    end      
    
    % Combine threshold across antennas
    threshold=mean(threshold)/sqrt(NRxAnts);          

    % Determine the length of the zero correlation zone. 
    if (info.NCS==0)
        zcz = 0;
    else
        zcz = (info.NCS/info.NZC)*info.SamplingRate/info.SubcarrierSpacing;
    end   
                
    % The following parameter specifies the fraction of the timing window at
    % the end of the timing window for one preamble that will be considered 
    % as belonging to the next preamble and having a timing offset of zero. 
    % This effectively excludes timing offsets of above (1.0-deadzone) of the 
    % maximum and ensures detection of preambles with low timing offset where 
    % noise has caused the peak of the correlation to be slightly into the 
    % previous preamble's timing window. The value configured below corresponds
    % to the duration of the main lobe of the autocorrelation of the PRACH.
    % (zero is used for the case that NCS=0 as there is only one preamble 
    %  per correlation.)    
    if (zcz~=0)
        deadzone=info.SamplingRate/(info.NZC*info.SubcarrierSpacing)/zcz;
    else
        deadzone=0;
    end
            
    % Detect preambles. The implementation here will detect a single
    % preamble index with the strongest correlation across all correlators,
    % provided a detection threshold is exceeded. 
    linearidx=0:duration-1;
    bestCorr=[];
    % Given values

	% figure;
%      stem(pdp,'b', 'LineWidth', 2);xlines(cyclicShift);yline(threshold,'--r','Threshold','LineWidth',1);hold on;title('Power profile delay'); legend('PDP','Bin','Threshold'); 

    % For each unique root sequence
    for i=1:length(u)        

    % If the maximum value of the correlation for this root sequence
    % exceeds the detection threshold and is the highest maximum value
    % across correlations checked so far

    %         if (max(c{i})>threshold)
    %         if (max(c{i})>bestCorr)

            % Record the peak correlation value and its position.
%             bestCorr=max(c{i});            
%             maxpos=mod(linearidx(c{i}==max(c{i}))+(deadzone*zcz),length(c{i}))-(deadzone*zcz);  

        % Establish the preamble index and timing offset from 
        % the correlation peak position.
        if (info.NCS==0)
            indout = preambleidx(i);
            offset = maxpos;
        else
            % Find the set of cyclic shifts v=0...maxv for this
            % root sequence.
            maxv=find(info.RootSeq==u(i),1,'Last')-preambleidx(i)-1;
            cyclicShift=(mod(info.NZC-info.CyclicShift(preambleidx(i)+(1:(maxv+1))),info.NZC)/info.NZC)*info.SamplingRate/info.SubcarrierSpacing;

            %[picos, bestCorr] = find_peaks_above_threshold(pdp, threshold);                    % Establish the set of offsets from the peak correlation
            min_peak_distance = 3;
            [picos, bestCorr] = find_multiple_peaks(pdp, threshold, min_peak_distance);
            Untitled6 (pdp, picos, bestCorr, threshold, cyclicShift)
            % position to the set of cyclic shifts for this root
            % sequence.
            % Plot the peaks on the PDP
            indout = [];
            offset = [];
            for ck = 1:length(picos)
                maxpos =(picos(ck));
                offsetpos=maxpos-cyclicShift;
                if (config.HighSpeed)
                % For High Speed mode, determine the size of the 
                % cyclic offset due to Doppler shift, in order to 
                % deal with the side peaks caused by loss of 
                % orthogonality due to high Doppler shift.               
                cyclicOffset=(info.CyclicOffset(preambleidx(i)+1)/info.NZC)*info.SamplingRate/info.SubcarrierSpacing;                        
                offsetpos=[offsetpos-cyclicOffset offsetpos offsetpos+cyclicOffset]; %#ok<AGROW>
                end

                % Find the value of v for the detected preamble and
                % compute the final preamble index.
                vdash=find(floor(offsetpos/zcz + deadzone)==0)-1;
                if (~isempty(vdash))
                    vdash=vdash(1);
                end
                v=mod(vdash,maxv+1);
                indout_t = preambleidx(i)+v;   
                indout = [indout indout_t];
                % Establish the timing offset from the correlation peak position.
                offset_t = offsetpos(vdash+1);
                offset = [offset offset_t];
                if (offset<0)
                    offset = 0;
                end         
            end
        end           
    end
    
    % Remove output values if detected preamble is not part of input
    % preamble index range INDIN.
    if(~isempty(indout))
        if(isempty(find(indin==indout,1)))
            indout=[];
            offset=[];
        end
    end
    
end
