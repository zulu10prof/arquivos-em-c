// DM40_Test.cpp â€” teste simples do DMP40 variando Z1 (Noll=1) passo a passo
#include "TestFunctions.h"
#include <vector>
#include <iostream>
#include <conio.h>

int dm40()
{
    std::cout << "[TRACE] Enter dm40 (teste de coeficiente Z1 no DMP40)" << std::endl;

    if (IsSimulate())
    {
        std::cout << "[SIM] dm40: incrementando Z1 em passos de 0.1 (offline). Pressione 'x' para sair." << std::endl;
        double val = 0.0;
        while (true)
        {
            if (_kbhit()) { char c = _getch(); if (c=='x' || c=='X') break; }
            std::cout << "[SIM] Z1=" << val << std::endl;
            val += 0.1;
            Sleep(150);
        }
        return Rtn_OK;
    }

    // Cria e configura sistema apenas com DM40 (controlador)
    if (create_DMP40_DM_ControlSystem() == Rtn_ERROR)
    {
        std::cerr << "[Erro] dm40: nao foi possivel iniciar o sistema do DMP40." << std::endl;
        return Rtn_ERROR;
    }

    // Configura parametros do DMP40 e obtÃ©m os objetos de dados (inclui ArraySelectedZernikeTermAmplitudes)
    if (set_DMP40_DM_Parameters() == Rtn_ERROR)
    {
        std::cerr << "[Erro] dm40: falha ao configurar parametros do DMP40." << std::endl;
        closeControlSystem();
        return Rtn_ERROR;
    }
    if (get_DMP40_DM_DataObject() == Rtn_ERROR)
    {
        std::cerr << "[Erro] dm40: falha ao obter objetos de dados do DMP40." << std::endl;
        closeControlSystem();
        return Rtn_ERROR;
    }

    if (hControllerElement == NULL || hSelectedZernikeTermAmplitudes == NULL)
    {
        std::cerr << "[Erro] dm40: controlador/array de Zernike nao inicializados." << std::endl;
        closeControlSystem();
        return Rtn_ERROR;
    }

    // Capacidade e formato do buffer do DM40
    int count = iZernikeCoefficientCount;
    if (count <= 0)
    {
        count = GetDataPropertyInt(hSelectedZernikeTermAmplitudes, _DataPropertyInt::Data_Size1);
        ErrChk("GetDataPropertyInt");
    }
    int dm_dt  = GetDataPropertyInt(hSelectedZernikeTermAmplitudes, _DataPropertyInt::Data_Type);               ErrChk("GetDataPropertyInt");
    int dm_bpc = GetDataPropertyInt(hSelectedZernikeTermAmplitudes, _DataPropertyInt::Data_BytesPerComponent);  ErrChk("GetDataPropertyInt");
    int dm_cpd = GetDataPropertyInt(hSelectedZernikeTermAmplitudes, _DataPropertyInt::Data_ComponentsPerData);  ErrChk("GetDataPropertyInt");

    std::vector<double> zvec((size_t)count, 0.0);
    //std::cout << "tamanho do vetor=" << count << std::endl;
    SDKErrChk(LockElement(hControllerElement));

    double val = -1;
    std::cout << "[TRACE] dm40: iniciando varredura em Z1, passos de 0.1. Pressione 'x' para sair." << std::endl;
    while (true)
    {
        if (_kbhit()) { char c = _getch(); if (c=='x' || c=='X') break; }
        if (val>1) {break;}
        // Atualiza apenas o primeiro coeficiente (Noll 1 -> indice 0)
        std::fill(zvec.begin(), zvec.end(), 0.0);
        zvec[0] = val;

        if (dm_cpd <= 1 && dm_bpc == 4 && dm_dt == _DataType::Float)
        {
            std::vector<float> fvec((size_t)count, 0.0f);
            for (int i = 0; i < count; ++i) fvec[(size_t)i] = static_cast<float>(zvec[(size_t)i]);
            SDKErrChk(SetDataContent(hSelectedZernikeTermAmplitudes, fvec.data()));
        }
        else
        {
            SDKErrChk(SetDataContent(hSelectedZernikeTermAmplitudes, zvec.data()));
        }

        SDKErrChk(ExecuteApiFunction(hControllerElement, "TLDFMX_calculate_zernike_pattern"));
        SDKErrChk(ExecuteApiFunction(hControllerElement, "TLDFM_set_segment_voltages"));

        std::cout << "[DM40] AST45 (Noll 5)=" << val << " (demais coeficientes=0)" << std::endl;

        val += 0.2;
        Sleep(1000);
    }

    SDKErrChk(UnlockElement(hControllerElement));
    closeControlSystem();
    std::cout << "[TRACE] dm40: encerrado." << std::endl;
    return Rtn_OK;
}

/*
ExplicaÃ§Ã£o detalhada (linha a linha / por blocos)

CabeÃ§alho e includes
- TestFunctions.h: expÃµe handles globais (hControllerElement, hSelectedZernikeTermAmplitudes, iZernikeCoefficientCount) e utilitÃ¡rios (SDKErrChk/ErrChk).
- <vector>, <iostream>, <conio.h>: buffers temporÃ¡rios, logs no console, leitura de tecla (/_kbhit, _getch).

int dm40()
- Entra na funÃ§Ã£o de teste e loga a entrada.
- if (IsSimulate()): modo simulado (sem hardware). Imprime Z1 e incrementa 0.1 atÃ© usuÃ¡rio pressionar 'x'; retorna OK.

CriaÃ§Ã£o/configuraÃ§Ã£o do sistema DM40
- create_DMP40_DM_ControlSystem(): cria ControlSystem, nÃ³ raiz e adiciona o elemento "DMP40" ao nÃ³. Em caso de falha, retorna erro.
- set_DMP40_DM_Parameters(): ajusta parÃ¢metros do DMP40, inclusive SignalSel=1 (usar Zernike como sinal) e modo Realtime.
- get_DMP40_DM_DataObject(): obtÃ©m os DataHandles, incluindo "ArraySelectedZernikeTermAmplitudes" e preenche iZernikeCoefficientCount.

Sanidade de handles
- Verifica hControllerElement e hSelectedZernikeTermAmplitudes != NULL; se invÃ¡lidos, encerra e retorna erro.

Metadados do array de Zernike no DM40
- count = iZernikeCoefficientCount; fallback lÃª Data_Size1 se necessÃ¡rio.
- dm_dt, dm_bpc, dm_cpd: tipo/bytes/componentes do array para decidir entre float/double ao enviar.

LaÃ§o de varredura de Z1
- zvec: vetor double do tamanho de count, inicializado com zeros.
- LockElement(hControllerElement): bloqueia o elemento para escrever com seguranÃ§a.
- val inicia em 0.0; laÃ§o: sai em 'x'.
- Em cada iteraÃ§Ã£o: zera zvec, define zvec[0] = val (Noll 1 -> Ã­ndice 0); converte para float se necessÃ¡rio e chama SetDataContent.
- TLDFMX_calculate_zernike_pattern: gera o padrÃ£o de segmentos a partir dos coeficientes.
- TLDFM_set_segment_voltages: aplica as tensÃµes aos segmentos do DM40.
- Loga o valor de Z1 aplicado; incrementa val em 0.1; Sleep(150) para estabilizaÃ§Ã£o/visualizaÃ§Ã£o.

Encerramento
- UnlockElement(hControllerElement): libera o elemento controlado.
- closeControlSystem(): remove elementos/nÃ³ e fecha o ControlSystem.
- Loga tÃ©rmino e retorna Rtn_OK.
*/
