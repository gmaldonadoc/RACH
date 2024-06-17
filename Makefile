make:
	if test -f LTE-Sim; then rm LTE-Sim; fi;
	clear;
	./CONFIG/make_load-parameter-file.sh; 
	cd Debug; make clean; make; cd ..;
	ln -s Debug/LTE-Sim LTE-Sim;
	clear;
clean:
	rm LTE-Sim; cd Debug; make clean
	cd ..
	clear;
