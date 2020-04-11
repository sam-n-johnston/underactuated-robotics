
#     values, vectors = np.linalg.eig(A)
#     inv = np.linalg.inv(vectors)
#     beta = np.matmul(vectors, B)
#     print("straing...")
#     print(beta)
#     print("done...")

#     def isIContrallable(I):
#         if(np.amax(I) != 0. or np.amin(I) != 0.):
#             return True
#         else:
#             return False
    
#     def isControllable(matrix):
#         print(matrix)
#         isContrallable = True
#         for i in range(matrix.shape[0]):
#             if(not isIContrallable(matrix[i])):
#                 isContrallable = False
                
#         return isContrallable

#     print(isControllable(beta))