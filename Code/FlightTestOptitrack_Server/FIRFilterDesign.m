% parameters
N = 200-1;  % filter Order
passf = 1/20; % pass frequency in 1/#samples
order = 0; % order of derivatives, used for weighing stop band
weightadd = 0; % for second order, 1; else, 0.
N = N - order; % because adding derivative filter will add order
gd = 1/passf/2; % desired group delay in samples

% frequency samples
F1 = linspace(0,passf*0.5,20);  % passband
F2 = linspace(passf*0.55,passf*1.5,20); % transition band
F3 = linspace(passf*1.55,1,20); % stop band
F = [F1 F2 F3];

% amplitude values
H1 = exp(-1j*pi*F1*gd);
H2 = exp(-1j*pi*F2*gd - ((F2-passf*0.5)/(passf*0.5)).^2);
H3 = zeros(size(F3));
H = [H1 H2 H3];

%weight values
W = (F3/(1.5*passf)).^(order+weightadd);

%plot desired response
subplot(3,1,1);
scatter(F,abs(H))
subplot(3,1,2);
scatter(F,angle(H));

% compute optimized filter
f = fdesign.arbmagnphase('N,B,F,H',N,3, F1,H1, F2,H2, F3,H3); 
Hd_mnp = design(f,'firls','B3Weights', W, SystemObject = true);

% plot filter specs
subplot(3,1,3);
plot(Hd_mnp.Numerator);
hfvt = fvtool(Hd_mnp);

res=Hd_mnp.Numerator;