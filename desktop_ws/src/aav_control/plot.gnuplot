set datafile separator ","
set autoscale

files = system("ls -1 *.csv")
plot for [file in files] file u 1
#plot   "00001_p_0.750000_i_0.000000_d_-0.200000_01.cte.csv" using 1
#replot "00001_p_0.750000_i_0.000000_d_-0.200000_02.cte.csv" using 1
#replot "00001_p_0.750000_i_0.000000_d_-0.200000_03.cte.csv" using 1
#replot "00001_p_0.750000_i_0.000000_d_-0.200000_04.cte.csv" using 1
#replot "00001_p_0.750000_i_0.000000_d_-0.200000_05.cte.csv" using 1
#replot "00001_p_0.750000_i_0.000000_d_-0.200000_06.cte.csv" using 1
#replot "00001_p_0.750000_i_0.000000_d_-0.200000_07.cte.csv" using 1
#replot "00001_p_0.750000_i_0.000000_d_-0.200000_08.cte.csv" using 1
#replot "00001_p_0.750000_i_0.000000_d_-0.200000_09.cte.csv" using 1
#replot "00001_p_0.750000_i_0.000000_d_-0.200000_10.cte.csv" using 1

