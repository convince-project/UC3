# Data Acquisition Utilities

Questa cartella contiene script e strumenti per l'acquisizione dei dati del tour.

## Contenuto
- **recording.py**: quando un tour inizia registra *tempo del tour*, *tempo per poi*, *recoveries per poi*, *aborts per poi*. E appende i risultati al file *tour_times_without_detection.csv* o *tour_times_with_detection.csv* in base a ciò che viene specificato nel codice.
- **show.py**: mostra risultati statistici standard, come valori medi.
- **compute.py**: stampa i valori medi sul terminale.
- **compute_results.py**: crea e salva dentro la cartella *report_output* delle figure statisticamente rilevanti per mostrare i risultati del tour. 

## Note
- I dati acquisiti vengono salvati in formato CSV o altro formato standard.