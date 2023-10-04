function spd = LegSpd(dphi1,dphi4,phi1,phi4)
%LegSpd
%    SPD = LegSpd(DPHI1,DPHI4,PHI1,PHI4)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    2023-10-04 14:10:26

t2 = cos(phi1);
t3 = cos(phi4);
t4 = sin(phi1);
t5 = sin(phi4);
t6 = imag(t2);
t7 = real(t2);
t8 = imag(t4);
t9 = real(t4);
t10 = t2.*(1.9e+1./2.0e+2);
t11 = t3.*(1.9e+1./2.0e+2);
t12 = t4.*(1.9e+1./2.0e+2);
t13 = t5.*(1.9e+1./2.0e+2);
t21 = t2.*3.61e-2;
t22 = t3.*3.61e-2;
t23 = t4.*3.61e-2;
t24 = t5.*3.61e-2;
t14 = -t10;
t15 = -t13;
t16 = t6.*(1.9e+1./2.0e+2);
t17 = t7.*(1.9e+1./2.0e+2);
t18 = t8.*(1.9e+1./2.0e+2);
t19 = t9.*(1.9e+1./2.0e+2);
t25 = -t21;
t26 = -t23;
t27 = -t24;
t20 = -t17;
t28 = t12+t15;
t30 = t11+t14+1.9e+1./2.0e+2;
t31 = t23+t27;
t38 = t22+t25+3.61e-2;
t29 = t28.^2;
t32 = t30.^2;
t33 = t31.^2;
t34 = t2.*t28.*(1.9e+1./1.0e+2);
t35 = t3.*t28.*(1.9e+1./1.0e+2);
t36 = t4.*t30.*(1.9e+1./1.0e+2);
t37 = t5.*t30.*(1.9e+1./1.0e+2);
t39 = t2.*t31.*7.22e-2;
t40 = t3.*t31.*7.22e-2;
t41 = t38.^2;
t42 = t4.*t38.*7.22e-2;
t43 = t5.*t38.*7.22e-2;
t44 = t29+t32;
t47 = t34+t36;
t48 = t35+t37;
t45 = t44.^2;
t49 = t23+t47;
t50 = t24+t48;
t51 = t38+t44;
t54 = t44.*t47.*2.0;
t55 = t44.*t48.*2.0;
t46 = -t45;
t52 = 1.0./t51;
t56 = -t54;
t57 = -t55;
t53 = t52.^2;
t58 = t33+t41+t46;
t63 = t39+t42+t56;
t64 = t40+t43+t57;
t59 = sqrt(t58);
t60 = 1.0./t59;
t61 = t24+t26+t59;
t62 = t61.^2;
t65 = t52.*t61;
t88 = t49.*t53.*t61;
t89 = t50.*t53.*t61;
t90 = (t60.*t63)./2.0;
t91 = (t60.*t64)./2.0;
t66 = atan(t65);
t67 = t53.*t62;
t92 = -t90;
t93 = -t91;
t68 = t66.*2.0;
t69 = t67+1.0;
t94 = t21+t92;
t95 = t22+t93;
t70 = cos(t68);
t71 = sin(t68);
t76 = 1.0./t69;
t96 = t52.*t94;
t97 = t52.*t95;
t72 = imag(t70);
t73 = real(t70);
t74 = imag(t71);
t75 = real(t71);
t77 = t70.*(1.9e+1./1.0e+2);
t78 = t71.*(1.9e+1./1.0e+2);
t106 = t88+t96;
t107 = t89+t97;
t79 = t72.*(1.9e+1./1.0e+2);
t80 = t73.*(1.9e+1./1.0e+2);
t81 = t74.*(1.9e+1./1.0e+2);
t82 = t75.*(1.9e+1./1.0e+2);
t84 = t12+t78;
t86 = t10+t77-1.9e+1./4.0e+2;
t110 = t70.*t76.*t106;
t111 = t70.*t76.*t107;
t112 = t71.*t76.*t106;
t113 = t71.*t76.*t107;
t83 = -t80;
t85 = t84.^2;
t87 = t86.^2;
t98 = t16+t19+t79+t82;
t99 = t98.^2;
t100 = t18+t20+t81+t83+1.9e+1./4.0e+2;
t101 = t85+t87;
t102 = t100.^2;
t103 = 1.0./t100;
t105 = 1.0./sqrt(t101);
t104 = 1.0./t102;
t108 = t99+t102;
t109 = 1.0./t108;
spd = [(dphi4.*t105.*(t84.*t111.*(1.9e+1./2.5e+1)-t86.*t113.*(1.9e+1./2.5e+1)))./2.0+(dphi1.*t105.*(t84.*(t10-t110.*(1.9e+1./5.0e+1)).*2.0-t86.*(t12-t112.*(1.9e+1./5.0e+1)).*2.0))./2.0;-dphi1.*t102.*t109.*(t103.*(t17-t18+imag(t112).*(1.9e+1./5.0e+1)-real(t110).*(1.9e+1./5.0e+1))-t98.*t104.*(t16+t19-imag(t110).*(1.9e+1./5.0e+1)-real(t112).*(1.9e+1./5.0e+1)))+dphi4.*t102.*t109.*(t103.*(imag(t113).*(1.9e+1./5.0e+1)-real(t111).*(1.9e+1./5.0e+1))+t98.*t104.*(imag(t111).*(1.9e+1./5.0e+1)+real(t113).*(1.9e+1./5.0e+1)))];
end
