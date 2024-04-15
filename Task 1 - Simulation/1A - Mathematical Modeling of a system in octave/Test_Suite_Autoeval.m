1;
function settling_time = calculate_sett_time(t, y_in, y_setpoint)
	flag = 0;
	settling_time = 0;
	tog_old=0;
	for k = 1:length(t)
		error = y_setpoint - y_in(k);
		error_percent = (error/(y_setpoint-y_in(1)))*100;
		if abs(error_percent) < 100
			tog= error_percent/abs(error_percent);
			if tog!=tog_old
				flag = flag + 1;
				tog_old=tog;
			endif
		endif
		if abs(error_percent) > 100
			flag = 0;
		endif
		if flag > 5
			settling_time = 10;
			break
		endif
	endfor
endfunction

function [marks, remarks, status] = eval_AB ()
	Task_1A;
	Score = [0 0 0 0 0 0 0]; 
	Mp = 0.5 ;
	l = 0.15 ;
	g = -9.81 ;
	Ma = 0.25 ;
	r_a = 0.01;
	r_p = 0.01;
	Rp = 0.2 ;
	Ra = 0.1 ;
	tspan = 0:0.01:10;
	C =[1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
	D=[0;0;0;0];
	y0= [0; 0; 0; 0.1*pi];
	u = (0)*ones(size(tspan,2),1);
	I_arm = 0.00036 + (Ma*Ra)^2;
	I_pendulum_theta = 0.00073 + (Mp*(l/2))^2;
	I_pendulum_alpha = 0.00063 + (Mp*Rp)^2 ;
	[A,B] = Jacobian_A_B(Mp,l,g,Ma,Rp,Ra,I_arm,I_pendulum_theta,I_pendulum_alpha);
	[A1,B1] = Jacobian_A_B(Mp,l,-g,Ma,Rp,Ra,I_arm,I_pendulum_theta,I_pendulum_alpha);
	H = ss(A,B,C,D);
	H1 = ss(A1,B1,C,D);
	stu_K = lqr_Rotary_Inverted_Pendulum(A,B);
	Q=eye(4);
	R=1;
	sys= ss(A,B,C,D);
	our_K = lqr(sys,Q,R);
	G = ss(A-B*stu_K, B, C, D);
	y = lsim(H, u, tspan, y0);
	y1 = lsim(H1, u, tspan, y0);
	marks=0;
	remarks='';
	status='';
	try
		Score(1)=((size(A)==[4 4])&&(size(B)==[4 1]))*2;
		if Score(1)==2
			marks=marks+Score(1);
			remarks=strcat(remarks,' 1: Dimenssion of A & B verified. ');
		else
			Score(1)=0;
			marks=marks+Score(1);
			remarks=strcat(remarks,' 1: Dimenssion of A & B not as required verified. it should be [4 4] & [4 1]. ');
		endif
	catch
		Score(1)=0;
		marks=marks+Score(1);
		remarks=strcat(remarks,' 1: Error verifying the Dimenssion of A & B. ');
	end_try_catch
	try
		Score(2)=not(isstable(H))*4;
		if Score(2)==4
			marks=marks+Score(2);
			remarks=strcat(remarks,' 2: Open loop rotary inverted pendulum is unstable in nature. ');
		else
			Score(2)=0;
			marks=marks+Score(2);
			remarks=strcat(remarks,' 2: Open loop rotary inverted pendulum can not be stable in nature. Check Mathematical Model/State-Space Model. ');
		endif
	catch
		Score(2)=0;
		marks=marks+Score(2);
		remarks=strcat(remarks,' 2: Error checking the stability. ');
	end_try_catch
	try
		Score(3)=isctrb(H)*4;
		if Score(3)==4
			marks=marks+Score(3);
			remarks=strcat(remarks,' 3: System is controllable in nature. ');
		else
			Score(3)=0;
			marks=marks+Score(3);
			remarks=strcat(remarks,' 3: System should be controllable in order to develope the controller. Your system is not controllable. ');
		endif
	catch
		Score(3)=0;
		marks=marks+Score(3);
		remarks=strcat(remarks,' 3: Error checking the controllability. ');
	end_try_catch
	try
		Score(4)=isstabilizable(H)*4;
		if Score(4)==4
			marks=marks+Score(4);
			remarks=strcat(remarks,' 4: System is stabilizable in nature. ');
		else
			Score(4)=0;
			marks=marks+Score(4);
			remarks=strcat(remarks,' 4: System should be stabilizable in order to develope the controller. Your system is not stabilizable. ');
		endif
	catch
		Score(4)=0;
		marks=marks+Score(4);
		remarks=strcat(remarks,' 4: Error checking the stabilizability. ');
	end_try_catch
	try
		Score(5)=isstable(G)*4;
		if Score(5)==4
			marks=marks+Score(5);
			remarks=strcat(remarks,' 5: System is stable with LQR controller. A-B*K. ');
		else
			Score(5)=0;
			marks=marks+Score(5);
			remarks=strcat(remarks,' 5: System is not stable with LQR controller. A-B*K. ');
		endif
	catch
		Score(5)=0;
		marks=marks+Score(5);
		remarks=strcat(remarks,' 5: Error checking the stability of A-B*K. ');
	end_try_catch
	try
		Score(6)=isequal(our_K,stu_K)*2;
		if Score(6)==2
			marks=marks+Score(6);
			remarks=strcat(remarks,' 6: K matrix matched. ');
		else
			Score(6)=0;
			marks=marks+Score(6);
			remarks=strcat(remarks,' 6: K matrix did not match. Do not change anything in lqr_Rotary_Inverted_Pendulum(A,B) function. ');
		endif
	catch
		Score(6)=0;
		marks=marks+Score(6);
		remarks=strcat(remarks,' 6: Error matching K matrix with reference. ');
	end_try_catch
	try
		Score(7)=3*(calculate_sett_time(tspan', y(:,4), 0));
		if Score(7)==0
			remarks=strcat(remarks,' 7: Failed to achieve Open loop oscillation. Trying with different y. ');
			Score(7)=3*(calculate_sett_time(tspan', y1(:,4), 0));
		endif
		if Score(7)>0
			marks=marks+Score(7);
			remarks=strcat(remarks,' 7: Open loop oscillation achieved. ');
		else
			Score(7)=0;
			marks=marks+Score(7);
			remarks=strcat(remarks,' 7: Faied to achieve Open loop oscillation. ');
		endif
	catch
		Score(7)=0;
		marks=marks+Score(7);
		remarks=strcat(remarks,' 7: Error checking the Open loop oscillation. ');
	end_try_catch
	if marks<50
		status='FAILED';
	else
		status='SUCCESS';
	endif
endfunction