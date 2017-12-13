import subprocess
def makelist():
	result = subprocess.check_output('rostopic list', shell = True)
	result_tmp = list()
	string = ""
	j = 0
	for i in range(result.count('\n')):
		while result[j] != '\n':
			string += result[j]
			j += 1
		result_tmp.append(string)
		string = ""
		j += 1
	result = result_tmp
	return result