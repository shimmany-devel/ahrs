all:
	gcc -o ahrs_idle ahrs_idle.c -lpthread
	gcc -o ahrs_ping ahrs_ping.c -lpthread
	gcc -o ahrs_reset ahrs_reset.c -lpthread
	gcc -o ahrs_resume ahrs_resume.c -lpthread
	gcc -o ahrs_init ahrs_init.c -lpthread

clean:
	@rm -rf *.o ahrs_idle ahrs_ping ahrs_reset ahrs_resume ahrs_init
