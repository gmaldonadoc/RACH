function [Ms]= calculoMS (n,i,Nack,NPTmax,Mi,R,percent)
          euler= 2.718281;
          if n<1 || i<1
              Ms=0;
          else
                Msumrow=0;
                for xx=1:NPTmax
                    Msumrow=Msumrow+Mi(i,xx);
                end
                pn = calculopn (n,percent);
                Xi=0;
                for x=1:NPTmax
                    Xi=Xi+(Mi(i,x)*(pn)*euler^(-Msumrow/R));
                end
                 
              fone=Mi(i,n)*pn*euler^(-Msumrow/R);
              if Xi<=Nack
                Ms=fone;
              else
                Ms=(Nack*fone)/Xi;
              end
          end     
end