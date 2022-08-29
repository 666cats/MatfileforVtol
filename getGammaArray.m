function GammaArray=getGammaArray(ModelParams)

Jxx = ModelParams.Jxx;
Jxz = ModelParams.Jxz;
Jyy = ModelParams.Jyy;
Jzz = ModelParams.Jzz;

Gamma = Jxx*Jzz - Jxz*Jxz;
GammaArray.gamma1 = (Jxz*(Jxx-Jyy+Jzz))/Gamma;
GammaArray.gamma2 = (Jzz*(Jzz-Jyy)+Jxz*Jxz)/Gamma;
GammaArray.gamma3 = Jzz/Gamma;
GammaArray.gamma4 = Jxz/Gamma;
GammaArray.gamma5 = (Jzz-Jxx)/Jyy;
GammaArray.gamma6 = Jxz/Jyy;
GammaArray.gamma7 = ((Jxx-Jyy)*Jxx+Jxz*Jxz)/Gamma;
GammaArray.gamma8 = Jxx/Gamma;

end