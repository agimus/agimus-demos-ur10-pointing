import  pandas   as   pd
import math
import argparse
import  os
import yaml

typeOfCsvFiles = ["markers", "init", "model"]
tokensOfInterest = {
    typeOfCsvFiles[0] : "tapes_center_file",
    typeOfCsvFiles[1] : "init_points_file",
    typeOfCsvFiles[2] : "model_points_file"}
commentTokens = ["//", "#"]
separatorToken = ": "

# output files
typeOfOutputFiles = ["urdf", "srdf"]
outputFiles = {
    typeOfOutputFiles[0] : "urdf_output_path",
    typeOfOutputFiles[1] : "srdf_output_path"}

## Method parsing the config file in order to retrieve the type of features described in the csv files
# @param The path towards the config file that needs to be parse.
# @return A dictionnary containing the type of features and the list of corresponding csv files
def parseConfigFile(filename, tokensOfInterest, commentTokens, separatorToken):
    dictOfCsvFilesTypes = dict()
    filecontent = None
    try:
        with open(filename) as configFile:
            filecontent = configFile.readlines()
    except OSError as e:
        print(f"{type(e)}: {e}")
        return None
    for line in filecontent:
        ## Reading each line of the config line in order to find one of the token of interest
        for searchedToken in tokensOfInterest:
            if tokensOfInterest[searchedToken] not in line:
                pass
            else:
                posComment = len(line)
                for commentToken in commentTokens:
                    posCandidateCommentToken = line.find(commentToken)
                    posComment = min(posComment, posCandidateCommentToken) if posCandidateCommentToken >= 0 else posComment
                posSeparatorToken = line.find(separatorToken)
                if (posComment > posSeparatorToken and posSeparatorToken > 0):
                    ## Up to know we found:
                    ##  - a token we are interested in
                    ##  - a separator between the token and value
                    ##  - the potential comment token is not before the token of interest
                    ## We remove the token from the line to extract the value
                    ## i.e. we keep everything after the separator token
                    valueAndPossibleComment = line.split(separatorToken)[1]
                    value = valueAndPossibleComment[0: posComment - posSeparatorToken - len(separatorToken)]
                    value = value.strip(" \t\n")
                    if searchedToken in dictOfCsvFilesTypes:
                        ## The token was already found once, we concatenate the new value to the list
                        dictOfCsvFilesTypes[searchedToken].append(value)
                    else:
                        ## We had a list containing the found value to the dictionnary
                        dictOfCsvFilesTypes[searchedToken] = [value]
    return dictOfCsvFilesTypes

def  readFasteners ( filename, csvName, typeOfFeature ): 
     # read the csv file, indicate the separator and the header
    fasteners  =   pd . read_csv ( filename ,   sep = ',' ,usecols=["Fastener_Name","Xe", "Ye", "Ze", "Xdir", "Ydir", "Zdir", "Fastener_Diameter"],   header = 0 )
     # delete nan values
    fasteners  =   fasteners . dropna ( axis = 0 ,   how = 'any' )
    #Create a dictionary with the fastener name as key and the fastener data as value
    # write the dataframe to a file
    fasteners . to_csv ( filename+'_got' ,   index = False )
    fastenerDict  =   {}
    #Create a dictionary with the duplicated fastener names as key and the number of times the key is present as value
    duplicated_fastenerDict  =   {}
    # for each fastener in the csv file
    for   index ,   row   in   fasteners . iterrows ():
        # get fastener data with .to_dict()
        fastener_name = f"{csvName}-{row['Fastener_Name']}"
        if (fastener_name in duplicated_fastenerDict) or (fastener_name in fastenerDict):
          id = 1
          if fastener_name in duplicated_fastenerDict:
            id = duplicated_fastenerDict[fastener_name] + 1
          duplicated_fastenerDict[fastener_name] = id
          fastener_name = fastener_name + "-" + str(id)
        fastenerDict [  fastener_name ]   =   row . to_dict ()  
        fastenerDict [ fastener_name ][ 'csvName' ]   =   csvName
        fastenerDict [ fastener_name ][ 'type' ]   =   typeOfFeature
        fastenerDict [ fastener_name ][ 'color' ]   =   "Orange" if typeOfFeature == "markers" else "Black"
    return   fastenerDict

def createURDF(fastenerDict, filename, formats):
    # open the file
    f = open(filename, 'w')
    # write the header of the file
    f.write(formats['urdf_header_format'])
    # for each fastener in the dictionary write the format color
    for fastener in fastenerDict:
        f.write(formats['urdf_format_color'].format(link_name= f"{fastenerDict[fastener]['type']}_{fastener}_link", color=fastenerDict[fastener]['color']))
        
    # write the part format
    f.write(formats['urdf_part_format'])
    # for each fastener in the dictionary write the format position
    for fastener in fastenerDict.keys():
        # get the type of fastener :model, init or marker
        link_name = f"{fastenerDict[fastener]['type']}_{fastener}_link"
        to_rivet = f"to_{fastenerDict[fastener]['type']}_{fastener}"
        x = float(fastenerDict[fastener]['Xe']) /1000
        y = float(fastenerDict[fastener]['Ye']) / 1000
        z = float(fastenerDict[fastener]['Ze']) / 1000
        
        # get euler angle from CATIA UDF Xdir, Ydir, Zdir
        Xdir = float(fastenerDict[fastener]['Xdir'])
        Ydir = float(fastenerDict[fastener]['Ydir'])
        Zdir = float(fastenerDict[fastener]['Zdir'])
        rx = - math.atan2(Ydir, Zdir) + math.pi
        ry = math.atan2(Xdir, math.sqrt(Ydir**2 + Zdir**2))
        rz = 0

        # Correct x axis
        x+= 2.53945
        if fastenerDict[fastener]['type'] == "markers":
            f.write(formats['urdf_marker_format'].format(link_name=link_name, joint_name=to_rivet, x=x, y=y, z=z, rx=rx, ry=ry, rz=rz))
        else :
            # get fastener diameter (csv unit in mm)
            radius = float(fastenerDict[fastener]['Fastener_Diameter']) / 1000 / 2
            f.write(formats['urdf_rivet_format'].format(link_name=link_name, to_rivet=to_rivet, x=x, y=y, z=z, rx=rx, ry=ry, rz=rz, radius=radius))

    # write the footer of the file
    f.write(formats['urdf_footer_format'])
    # close the file
    f.close()

# Format used to create the srdf file
# Function that create associated srdf file for the fasteners
def  createFastenerSrdf ( fastenerDict,   filename, formats ): 
    # open the file
    f   =   open ( filename ,   'w' )
    f . write ( '<robot name="aircraft">' )
    # for each fastener in the dictionary
    for   fastener   in   fastenerDict.keys():
        # write the handle tag
        link_name = fastenerDict[fastener]['type'] +  "_"   +   fastener   +   "_link"
        name = "handle_"   +   fastener
        
        f . write ( formats['srdf_format'] . format ( name = name , link_name = link_name ))
        
    # write the end of the file
    f . write ( '</robot>' )
    # close the file
    f . close ()


def main(input_files, csv_directory, urdf_filename, srdf_filename, formats):
  # get the fasteners from the csv files with readFasteners() and assign a color for each file picked up in the colors list
  # Do the file wich contains "Patterns4Initialisation" in his name last
  fasteners =  {}
  fastenerCount = 0
  # for loop with enumerate to get the index of the file
  for  index ,   fastenerFileType   in   enumerate ( input_files ):
    for inputFile in input_files[fastenerFileType]:
      fastenerFile = os.path.join(csv_directory, inputFile)
      # get the name of the file without the extension
      csvName   =   os . path . basename ( fastenerFile ) . split ( '.' )[ 0 ]
      # Get the "D**-STD**" in csv name to create fastener unique name : REACT_D53118641000-STD60_20230131_HnF_Export -> D53118641000-STD60
      csvName = csvName.split('_')[1]
      # get the fasteners from the csv file and append them to the dictionary
      fileFastenersDict = readFasteners(fastenerFile, csvName=csvName, typeOfFeature=fastenerFileType)
      fastenerCount += len(fileFastenersDict)
      fasteners.update(fileFastenersDict)

      print ( f"File {csvName} of type {fastenerFileType} has {len(fileFastenersDict)} fasteners" )
  # print() the length of the dictionary to see how many fasteners we have with f"{}" format 
  print ( f"Total number of fasteners in cvs: {fastenerCount}" )
  print ( f"Total final number of fasteners:  {len(fasteners)} -> {fastenerCount - len(fasteners)} duplicates (same name and same csv filename).")

  createURDF(fasteners, urdf_filename, formats)
  createFastenerSrdf(fasteners, srdf_filename, formats)


if __name__ == "__main__":
  import sys

  # get the current directory
  currentDir   =   os . getcwd ()

  parser = argparse.ArgumentParser(epilog = 'Usage: python3 %(prog)s --config_file /tmp/config.txt --csv_dir /tmp/fasteners --urdf_file /tmp/urdf/aircraft.urdf --srdf_file /tmp/srdf/aircraft.srdf')
  parser.add_argument('--config_file', type=str, default=os.path.join(currentDir,'config.txt'), help="Configuration file listing the different csf files and their types (input)")
  parser.add_argument('--csv_dir', type=str, default=os.path.join(currentDir,'fasteners'), help="Directory in which are located the csv files (input)")
  # parser.add_argument('--urdf_file', type=str, default=os.path.join(currentDir,'urdf', 'aircraft.urdf'), help="Path of the resulting urdf file (output)")
  # parser.add_argument('--srdf_file', type=str, default=os.path.join(currentDir,'srdf', 'aircraft.srdf'), help="Path of the resulting srdf file (output)")
  # yaml file formats.yaml
  parser.add_argument('--formats_file', type=str, default=os.path.join(currentDir,'formats.yaml'), help="Path of the formats.yaml file (input)")
  args = parser.parse_args()


  # get default urdf and srdf output files from config file. ex: {'srdf': ['../srdf/aircraft.srdf'], 'urdf': ['../urdf/aircraft.urdf']}
  output_files = parseConfigFile(args.config_file,tokensOfInterest=outputFiles,separatorToken=separatorToken,commentTokens=commentTokens)
  urdf_filename = os.path.join(currentDir,output_files['urdf'][0])
  srdf_filename = os.path.join(currentDir,output_files['srdf'][0])
  

  if not os.path.exists(args.config_file):
        print("Error: Input config file \"", args.config_file, "\" does not exist!")
        sys.exit(1)
  
  if not os.path.exists(args.csv_dir):
        print("Error: Input csv directory \"", args.csv_dir, "\" does not exist!")
        sys.exit(1)

  if os.path.exists(urdf_filename):
        print("Error: Output urdf file \"", urdf_filename, "\" exists!")
        # ask the user if he wants to overwrite the file
        if input("Do you want to overwrite the file? (y/n)") == 'n':
          sys.exit(1)

  if os.path.exists(srdf_filename):
        print("Error: Output srdf file \"", srdf_filename, "\" exists!")
        # ask the user if he wants to overwrite the file
        if input("Do you want to overwrite the file? (y/n)") == 'n':
          sys.exit(1)
  if not os.path.exists(args.formats_file):
        print("Error: Input formats file \"", args.formats_file, "\" does not exist!")
        sys.exit(1)

  # Load the formats from the YAML file
  with open(args.formats_file, 'r') as f:
    formats = yaml.safe_load(f)

  # get the csv files listed in the config file, along with the type of features they describe
  input_files = parseConfigFile(args.config_file,tokensOfInterest=tokensOfInterest,separatorToken=separatorToken,commentTokens=commentTokens)
  main(urdf_filename= urdf_filename, srdf_filename=srdf_filename, csv_directory=args.csv_dir, input_files = input_files, formats = formats)
