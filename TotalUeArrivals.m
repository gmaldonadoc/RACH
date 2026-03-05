% clear all;clc;clear vars
% R=54;
% percent =0.99;
% RAOS= 2100; % Nº de RAO
% NUEcell=30000;
%variables
function [sumrow,Mi]= TotalUeArrivals (NUEcell,R,RAOS)
percent =0.99;
Trar = 2;
Wrar = 5;
Nrar=3;
t1= 0;
Tra_rep= 5;
TP= 10000;
BI=20;
NPTmax= 10;
Wbo=BI+1;

K1 = ceil((Trar+Wrar)/Tra_rep);
K2 = 1 + floor((Trar+Wrar+Wbo)/Tra_rep);
xa = ((ceil((Trar+Wrar)/Tra_rep)*Tra_rep)-(Trar+Wrar))/Wbo;
xbc= Tra_rep/Wbo;
xd= ((Trar+Wrar+Wbo)/Wbo) - (Tra_rep/Wbo)*floor((Trar+Wrar+Wbo)/Tra_rep);
Nack = Nrar*Wrar;
if mod((TP-t1),(Tra_rep))==0
    TE=0;
else 
    TE= Tra_rep- mod((TP-t1),(Tra_rep));
end
TRAmax = 1+((NPTmax-1)*ceil((Trar+Wrar+Wbo)/Tra_rep)*Tra_rep);
TRAI = TP+TE+TRAmax;
IR = floor(TRAI/Tra_rep);
Mi= zeros(1,RAOS)';
MS= zeros(1,RAOS)';
MColission = zeros(1,RAOS);
MSuccess = zeros(1,RAOS)';
for i = 1: RAOS 

    for n = 1:NPTmax
        if n == 1
            Mi(i,n) = ((NUEcell*60*(i^2))*(RAOS-i)^3)/(RAOS^6); % matriz
        elseif  i==1 
            Mi(i,n) =0;
        else

          kmin = i - K2 + 1;
          kmax = i - K1 - 1;
          n1 = n-1;
          i1 = i-K1;
          i2 = i-K2;

             sumk=0;
             for k=kmin:kmax
                sumk=sumk+xbc*calculoMC(n1,k,Nack,NPTmax,Mi,R,percent);
             end

          mcf1=calculoMC(n1,i1,Nack,NPTmax,Mi,R,percent);
          mcf2=calculoMC(n1,i2,Nack,NPTmax,Mi,R,percent);
          Mi(i,n)= (xa*mcf1)+(xd*mcf2)+sumk;

        end
    end
end
    sumrow = sum (Mi,2);

    for ii = 1: RAOS 
        for nn = 1:NPTmax
            MS(ii,nn) = calculoMS(nn,ii,Nack,NPTmax,Mi,R,percent);
        end
    end

    Msumtotal=0;
    for y=1:IR
         Msum=0;
        for g=1:NPTmax
            Msum=Msum+MS(y,g);
        end
        Msumtotal=Msumtotal+Msum;
    end
    Ps=Msumtotal/NUEcell;
end