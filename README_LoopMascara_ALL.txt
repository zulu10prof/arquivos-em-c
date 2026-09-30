LoopMascara — README unico (DMP40 e BMC 140)

Objetivo
- Em loop, gerar fase turbulenta (modelo Kolmogorov), projetar em base de Zernike (indexacao de Noll) e acionar o espelho deformavel.
- Suporta dois alvos: DMP40 (Thorlabs) e BMC 140. Parar com tecla 'x' ou ESC. Em simulacao, apenas mensagens sao exibidas.

Como executar
- DMP40: menu "j: Loop mascara de fase".
- BMC 140: menu "l: Loop mascara BMC 140".
- Para atuar no hardware: THOR_SIMULATE=0 (VS: Debugging > Environment; CMD: set THOR_SIMULATE=0).

Pre-requisitos
- DMP40: plugin/SDK do DMP40 instalados; dispositivo conectado.
- BMC 140: plugin/SDK BMC Mini/Multi instalado; dispositivo conectado.
- Projeto compila e inicializa o controle sem erros.

Parametros principais (valores padrao no codigo)
- Ngrid = 32                     (resolucao da malha sobre a pupila)
- K = min(12, iZernikeCoefficientCount) (quantidade de modos Zernike usados)
- lambda_um = 0.6328             (comprimento de onda em micrometros)
- r0_relD = tipicamente 0.2..2   (intensidade de turbulencia relativa ao diametro)
- numCycles, numModes             (sintese da fase; ex.: 10/100 DMP40, 20/400 BMC)
- amplitude_frac = 1             (BMC: fracao do range [offset..max] para modulacao)

Relacoes e formulas
- Kolmogorov (estatistica da fase):
  D_phi(rho) = 6.88 * (rho / r0)^(5/3);
  Phi_phi(k) = 0.023 * r0^(-5/3) * |k|^(-11/3) (PSD).
- Zernike e Noll: j -> (n,m); Z_j(r,theta) = Z_n^m(r,theta).
- Minimos quadrados: (A^T A + lambda I) c = A^T b (Tikhonov leve) com
  A[p,j] = Z_j(r_p,theta_p) e b_p = phi(r_p,theta_p).
- Fase [rad] e OPD [um]: phi = (2*pi/lambda) * OPD => OPD = phi * (lambda/2*pi).
  Para espelho refletivo e deslocamento de superficie h, OPD = 2*h.

Fluxo — DMP40 (LoopMascara.cpp)
1) Inicializacao do controle (create/set/get...).
2) Pre-computo do cache: pixels da pupila (disco unitario); Z_j por pixel; ATA = A^T A.
3) Por iteracao:
   a) Gerar fase turbulenta (rad): generate_kolmogorov_phase_plane_waves(...).
   b) Calcular ATb e resolver (A^T A + lambda I) c = A^T b (eliminacao gaussiana, lambda~1e-8).
   c) Converter c de rad -> um: c_um = c_rad * (lambda_um / 2*pi).
   d) Preencher vetor Zernike do driver (tamanho iZernikeCoefficientCount); copiar primeiros K, zerar resto.
   e) Enviar sob lock: TLDFMX_calculate_zernike_pattern; TLDFM_set_segment_voltages.

Fluxo — BMC 140 (LoopMascara_BMC.cpp)
1) Inicializacao do BMC e leitura dos arrays de calibracao offset/max.
2) Por iteracao:
   a) Gerar fase turbulenta (rad): generate_kolmogorov_phase_plane_waves(...).
   b) Decompor nos K primeiros Zernike: decompose_phase_firstK_using_zernikePolynomial(...)
      (equivale a resolver (A^T A) c = A^T b, com Tikhonov leve conforme necessidade).
   c) Reconstruir fase na malha de atuadores (disco unitario) e calcular RMS para normalizacao.
   d) Mapear fase -> tensoes por atuador em [offset..max] com saturacao suave:
      scale = 3*RMS; norm = tanh(surf_rad/scale) em [-1,1];
      v = clamp(vctr + dv*norm, vmin=offset[i], vmax=max[i]), com vctr=(vmin+vmax)/2 e dv=0.5*amplitude_frac*(vmax-vmin).
   e) Enviar sob lock: SetDataContent(hArrayActuatorVoltages, ...); BMCSetArray.

Modo simulacao
- Se IsSimulate() retornar true, mensagens de [SIM] sao exibidas e o hardware nao e acionado.
- Para desativar simulacao: THOR_SIMULATE=0 (VS/CMD).

Ajustes e dicas
- K: aumentar reduz erro de projecao, mas respeite limites do dispositivo/calibracao.
- r0_relD, numCycles, numModes: controlam severidade/estatistica/velocidade da turbulencia.
- DMP40: verifique unidade esperada (OPD vs ondas vs superficie) antes de converter.
- BMC: ajuste amplitude_frac e scale (~3*RMS) para evitar saturacoes.
- Sleep: ajuste a taxa de atualizacao conforme resposta do hardware.

Erros comuns
- "Falha ao resolver normal equations" (DMP40): reduza K, verifique Ngrid e a mascara; pixels suficientes no disco.
- Sem acao no hardware: confirme THOR_SIMULATE=0, conexao/dispositivo OK e chamadas create/set/get retornando OK.

Referencias
- Noll, R. J. (1976). Zernike polynomials and atmospheric turbulence. JOSA.
- Fried, D. L. (1966). Optical resolution through a randomly inhomogeneous medium. JOSA.
- Goodman, J. W. (2000). Statistical Optics. Wiley.
- Roddier, F. (1999). Adaptive Optics in Astronomy. Cambridge Univ. Press.
- Born, M.; Wolf, E. (1999). Principles of Optics. Cambridge Univ. Press.
- Hardy, J. W. (1998). Adaptive Optics for Astronomical Telescopes. Oxford Univ. Press.

