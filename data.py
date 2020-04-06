# data.py
# Ben Webb
# Thesis

import csv, datetime, time, os.path
import numpy as np


class Data:
    # Object to store large amounts of data
    def __init__(self, filename = None):
        self.headers = []
        self.types = []
        self.data = None
        self.header2col = {}
        self.strings = []

        if filename != None:
            self.read(filename)

    # Reads CSV file
    def read(self, filename):

        with open(os.path.join('DataFolder/', filename), mode='rU') as fp:
            csv_reader = csv.reader(fp)

            self.headers = next(csv_reader)
            self.types = next(csv_reader)
            for i in range(len(self.headers)):
                self.headers[i] = self.headers[i].strip().lower()
                self.header2col[self.headers[i]] = i
                self.types[i] = self.types[i].strip().lower()

            datastream = []
            for line in csv_reader:
                datastream.append(line)
            self.data = np.matrix(datastream)

    def get_headers(self):
        # Returns all of the headers of the data
        return self.headers.copy()

    def get_types(self):
        # Returns all of the types of data
        return self.types.copy()

    def get_num_dimensions(self):
        # Returns the number of columns of data
        return self.data.shape[1]

    def get_row_size(self):
        # Returns the number of headers
        return len(self.headers)

    def get_num_points(self):
        # Returns the number of rows of data
        return self.data.shape[0]

    def count_matching(self, header, phrase):
        i = 0
        for j in range(self.get_num_points()):
            try:
                if phrase in self.data[self.header2col[header], j]: i += 1
            except:
                continue
        return i

    def get_row(self, rowIndex):
        # Returns a specific row of data
        return self.data[rowIndex]

    def get_value(self, header, rowIndex):
        # Returns a specific value, based on the row and column header
        if header == '':
            return -9999
        elif header not in self.header2col.keys():
            return None
        return self.data[rowIndex,self.header2col[header]]

    def get_data(self, headersInclude = (), headersExclude = (), rowsExclude = ()):
        # Returns a matrix of all of the specified headers with their columns of data
        tempHeaders = self.headers.copy()
        if headersExclude != ():
            for h in headersExclude:
                tempHeaders.remove(h)
        if headersInclude != ():
            tempHeaders = headersInclude
        matrix = []
        for h in tempHeaders:
            list = []
            for i in range(self.get_num_points()):
                if i in rowsExclude:
                    continue
                list.append([self.get_value(h, i)])
            matrix.append(np.matrix(list))
        return np.hstack(matrix.copy())

    def addColumn(self, header, type, column):
        # Adds a column of data to the data object
        if len(column) != self.get_num_points():
            print('Column is the wrong size!')
            return
        self.headers.append(header)
        self.header2col[header] = self.headers.index(header)
        self.types.append(type)
        self.data = np.hstack((self.data, column))

    def addRow(self, row=()):
        # Adds a Row of data to the data object
        if len(row) != self.get_row_size(): return
        if self.data.size == 0: self.data = np.mat(row)
        else: self.data = np.vstack((self.data, row))


    def write(self, filename=None, trial=True, Type = ''):
        if trial: path = os.path.join('DataFolder/Data/' + Type, filename)
        else: path = os.path.join('DataFolder/', filename)
        with open(path, 'w') as fp:
            csv_writer = csv.writer(fp)
            csv_writer.writerow(self.headers)
            csv_writer.writerow(self.types)
            data = np.asarray(self.data)
            for row in range(self.data.shape[0]):
                csv_writer.writerow(data[row])

class PCAData(Data):
    # Creates a Principle Component Analysis data object. Its a more particular version of the Data object
    def __init__(self, data = None, eVec = None, eVal = None, meanVal = None, dataHeaders = None):
        self.eVec = eVec
        self.eVal = eVal
        self.meanVal = meanVal
        Data.__init__(self)
        self.types = []
        self.dataHeaders = dataHeaders
        self.data = data
        for i in range(data.shape[1]):
            self.types.append('numeric')
            self.headerTocol['PCA%i'%(i)] = i
        self.headers = list(self.headerTocol.keys())

    def get_eigenvalues(self):
        # Returns the Eigen Values
        return self.eVal.copy()

    def get_eigenvectors(self):
        # Returns the Eigen Vectors
        return self.eVec.copy()

    def get_original_means(self):
        # Returns the means
        return self.meanVal.copy()

    def get_headers(self):
        # Returns the headers (PCA###)
        return self.headers.copy()

    def get_original_headers(self):
        # Returns the original headers used to create the PCA
        return self.dataHeaders.copy()


if __name__ == "__main__":
    data = Data('TableOfContents.csv')
    data.addRow(('Line.csv', '/DataFolder/Data', time.time(), 'line', 10, 1, 1, 1, 1, 2, 2, 3))
    data.write('TableOfContents.csv', False)
    # print(data.get_num_dimensions())
    # print(data.get_num_points())
    # print(data.get_row(1))
    # data.write('writeclusterdata.csv', data.get_headers())
