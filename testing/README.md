# Testing Scripts

- run_tests.py: runs tests on a range of number of convoy robots (given as a list) collecting all data recorded
- aggregate_data.py: helper script for running tests and aggregating the data removing the unnecessary duplicates occuring due to some multi-threaded features of Stage
- evaluate.py: produces histograms and statistical measures from the aggregated data
- evaluate_crashing.py: produces a graph displaying the crash percentage of the algorithms against the number of convoy robots
