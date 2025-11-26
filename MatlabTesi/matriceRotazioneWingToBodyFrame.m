function Rwb = matriceRotazioneWingToBodyFrame(alpha,beta)
% matrice per il cambio di coordinate da wing frame a body frame
sa = sin(alpha);
ca = cos(alpha);
sb = sin(beta);
cb = cos(beta);

Rwb = [ca*cb -ca*sb -sa; sb cb 0; sa*cb -sa*sb ca];

end