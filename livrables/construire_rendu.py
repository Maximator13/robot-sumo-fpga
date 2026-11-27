"""Construit livrables/robot_sumo_rendu.zip, le ZIP de rendu du projet robot SUMO.

Contenu (CDC §2 : projet Quartus + fichiers VHDL + document de mise en oeuvre) :
sources VHDL, bancs de test, scripts de simulation, contraintes, projet Quartus
(.qpf, .qsf, .sof, .jic), document d'architecture PDF, et les deux fichiers UART
d'origine de l'enseignant (pour sim/check_uart_origine.sh).
Les fichiers de travail (journal d'avancement, notes et fiches de révision)
ne sont volontairement PAS inclus.

Usage, depuis la racine du projet :  python livrables/construire_rendu.py
"""
import os
import zipfile

RACINE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SORTIE = os.path.join(RACINE, 'livrables', 'robot_sumo_rendu.zip')

contenu = []
for dossier, extension in [('src', '.vhd'), ('tb', '.vhd'), ('constraints', '')]:
    for nom in sorted(os.listdir(os.path.join(RACINE, dossier))):
        if nom.endswith(extension):
            contenu.append(dossier + '/' + nom)
contenu += [
    'sim/run_tb.sh', 'sim/run_all.sh', 'sim/modules.lst', 'sim/check_uart_origine.sh',
    'quartus/robot_sumo.qpf', 'quartus/robot_sumo.qsf',
    'quartus/output_files/robot_sumo.sof', 'quartus/output_files/robot_sumo.jic',
    'livrables/architecture.pdf',
    'docs/Projet_FPGA_2026/VHDL_UART/UART_RX.vhd',
    'docs/Projet_FPGA_2026/VHDL_UART/UART_TX.vhd',
]

manquants = [f for f in contenu if not os.path.isfile(os.path.join(RACINE, f))]
if manquants:
    raise SystemExit('Fichiers manquants (compiler le projet avant ?) : ' + ', '.join(manquants))

with zipfile.ZipFile(SORTIE, 'w', zipfile.ZIP_DEFLATED) as z:
    for rel in contenu:
        z.write(os.path.join(RACINE, rel), 'robot_sumo/' + rel)
print(len(contenu), 'fichiers ->', SORTIE)
