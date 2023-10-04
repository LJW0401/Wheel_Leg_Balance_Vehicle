function T = LegConv(F,Tp,phi1,phi4)
%LegConv
%    T = LegConv(F,Tp,PHI1,PHI4)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    2023-10-04 14:10:28

t2 = conj(phi1);
t3 = conj(phi4);
t4 = cos(phi1);
t5 = cos(phi4);
t6 = sin(phi1);
t7 = sin(phi4);
t8 = cos(t2);
t9 = cos(t3);
t10 = imag(t4);
t11 = real(t4);
t12 = sin(t2);
t13 = sin(t3);
t14 = imag(t6);
t15 = real(t6);
t16 = t4.*(1.9e+1./2.0e+2);
t17 = t5.*(1.9e+1./2.0e+2);
t18 = t6.*(1.9e+1./2.0e+2);
t19 = t7.*(1.9e+1./2.0e+2);
t33 = t4.*3.61e-2;
t34 = t5.*3.61e-2;
t35 = t6.*3.61e-2;
t36 = t7.*3.61e-2;
t20 = -t16;
t21 = -t19;
t22 = t8.*(1.9e+1./2.0e+2);
t23 = t9.*(1.9e+1./2.0e+2);
t24 = t10.*(1.9e+1./2.0e+2);
t25 = t11.*(1.9e+1./2.0e+2);
t26 = t12.*(1.9e+1./2.0e+2);
t27 = t13.*(1.9e+1./2.0e+2);
t28 = t14.*(1.9e+1./2.0e+2);
t29 = t15.*(1.9e+1./2.0e+2);
t37 = -t33;
t38 = -t35;
t39 = -t36;
t40 = t8.*3.61e-2;
t41 = t9.*3.61e-2;
t42 = t12.*3.61e-2;
t43 = t13.*3.61e-2;
t30 = -t22;
t31 = -t25;
t32 = -t27;
t44 = -t40;
t45 = -t42;
t46 = -t43;
t47 = t18+t21;
t51 = t17+t20+1.9e+1./2.0e+2;
t52 = t35+t39;
t64 = t34+t37+3.61e-2;
t48 = t47.^2;
t49 = t26+t32;
t53 = t51.^2;
t54 = t52.^2;
t55 = t23+t30+1.9e+1./2.0e+2;
t56 = t42+t46;
t57 = t4.*t47.*(1.9e+1./1.0e+2);
t58 = t5.*t47.*(1.9e+1./1.0e+2);
t62 = t6.*t51.*(1.9e+1./1.0e+2);
t63 = t7.*t51.*(1.9e+1./1.0e+2);
t65 = t4.*t52.*7.22e-2;
t66 = t5.*t52.*7.22e-2;
t67 = t64.^2;
t68 = t41+t44+3.61e-2;
t73 = t6.*t64.*7.22e-2;
t74 = t7.*t64.*7.22e-2;
t50 = t49.^2;
t59 = t55.^2;
t60 = t8.*t49.*(1.9e+1./1.0e+2);
t61 = t9.*t49.*(1.9e+1./1.0e+2);
t69 = t12.*t55.*(1.9e+1./1.0e+2);
t70 = t13.*t55.*(1.9e+1./1.0e+2);
t71 = t8.*t56.*7.22e-2;
t72 = t9.*t56.*7.22e-2;
t75 = t12.*t68.*7.22e-2;
t76 = t13.*t68.*7.22e-2;
t77 = t48+t53;
t81 = t57+t62;
t82 = t58+t63;
t78 = t77.^2;
t79 = t50+t59;
t83 = t60+t69;
t84 = t61+t70;
t85 = t35+t81;
t86 = t36+t82;
t89 = t64+t77;
t95 = t77.*t81.*2.0;
t96 = t77.*t82.*2.0;
t80 = -t78;
t87 = t42+t83;
t88 = t43+t84;
t90 = 1.0./t89;
t92 = t68+t79;
t97 = -t95;
t98 = -t96;
t103 = t79.*t83.*2.0;
t104 = t79.*t84.*2.0;
t91 = t90.^2;
t93 = 1.0./t92;
t99 = t54+t67+t80;
t105 = -t103;
t106 = -t104;
t112 = t65+t73+t97;
t113 = t66+t74+t98;
t94 = t93.^2;
t100 = sqrt(t99);
t114 = t71+t75+t105;
t115 = t72+t76+t106;
t101 = conj(t100);
t102 = 1.0./t100;
t108 = t36+t38+t100;
t107 = 1.0./t101;
t109 = t108.^2;
t110 = t43+t45+t101;
t116 = t90.*t108;
t150 = t85.*t91.*t108;
t151 = t86.*t91.*t108;
t152 = (t102.*t112)./2.0;
t153 = (t102.*t113)./2.0;
t111 = t110.^2;
t117 = atan(t116);
t119 = t91.*t109;
t154 = t87.*t94.*t110;
t155 = t88.*t94.*t110;
t156 = -t152;
t157 = -t153;
t160 = (t107.*t114)./2.0;
t161 = (t107.*t115)./2.0;
t118 = conj(t117);
t120 = t117.*2.0;
t121 = t119+1.0;
t134 = t94.*t111;
t158 = t33+t156;
t159 = t34+t157;
t162 = -t160;
t163 = -t161;
t122 = t118.*2.0;
t123 = cos(t120);
t124 = sin(t120);
t131 = 1.0./t121;
t142 = t134+1.0;
t164 = t40+t162;
t165 = t41+t163;
t166 = t90.*t158;
t167 = t90.*t159;
t125 = cos(t122);
t126 = imag(t123);
t127 = real(t123);
t128 = sin(t122);
t129 = imag(t124);
t130 = real(t124);
t132 = t123.*(1.9e+1./1.0e+2);
t133 = t124.*(1.9e+1./1.0e+2);
t143 = 1.0./t142;
t168 = t93.*t164;
t169 = t93.*t165;
t180 = t150+t166;
t181 = t151+t167;
t135 = t125.*(1.9e+1./1.0e+2);
t136 = t126.*(1.9e+1./1.0e+2);
t137 = t127.*(1.9e+1./1.0e+2);
t138 = t128.*(1.9e+1./1.0e+2);
t139 = t129.*(1.9e+1./1.0e+2);
t140 = t130.*(1.9e+1./1.0e+2);
t144 = t18+t133;
t147 = t16+t132-1.9e+1./4.0e+2;
t182 = t154+t168;
t183 = t155+t169;
t186 = t123.*t131.*t180;
t187 = t123.*t131.*t181;
t188 = t124.*t131.*t180;
t189 = t124.*t131.*t181;
t141 = -t137;
t145 = t144.^2;
t146 = t26+t138;
t148 = t22+t135-1.9e+1./4.0e+2;
t149 = t147.^2;
t170 = t24+t29+t136+t140;
t171 = t170.^2;
t172 = t28+t31+t139+t141+1.9e+1./4.0e+2;
t173 = t145+t149;
t174 = t172.^2;
t175 = 1.0./t172;
t177 = sqrt(t173);
t176 = 1.0./t174;
t178 = conj(t177);
t184 = t171+t174;
t179 = 1.0./t178;
t185 = 1.0./t184;
T = [(F.*t179.*(t146.*(t22-t125.*t143.*t182.*(1.9e+1./5.0e+1)).*2.0-t148.*(t26-t128.*t143.*t182.*(1.9e+1./5.0e+1)).*2.0))./2.0-Tp.*t174.*t185.*(t175.*(t25-t28+imag(t188).*(1.9e+1./5.0e+1)-real(t186).*(1.9e+1./5.0e+1))-t170.*t176.*(t24+t29-imag(t186).*(1.9e+1./5.0e+1)-real(t188).*(1.9e+1./5.0e+1)));(F.*t179.*(t125.*t143.*t146.*t183.*(1.9e+1./2.5e+1)-t128.*t143.*t148.*t183.*(1.9e+1./2.5e+1)))./2.0+Tp.*t174.*t185.*(t175.*(imag(t189).*(1.9e+1./5.0e+1)-real(t187).*(1.9e+1./5.0e+1))+t170.*t176.*(imag(t187).*(1.9e+1./5.0e+1)+real(t189).*(1.9e+1./5.0e+1)))];
end
