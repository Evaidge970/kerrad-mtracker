%skryot oblicza trasê przebyta przez oba ko³a, metod¹ prostok¹tów
dw = zeros(2,n);
for  i = 1:n
    dw(1,i) = w(1,i)*0.025*Ts;
    dw(2,i) = w(2,i)*0.025*Ts;
end

track = zeros(2,1);
for j = 1:n
    track(1) = track(1) + dw(1,j);
    track(2) = track(2) + dw(2,j);
end
    

    
    