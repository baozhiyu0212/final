import smtplib  
import email.MIMEMultipart  
from email.mime.text import MIMEText
import email.MIMEBase  
import os.path

def alertMe():
	From = "bayu9537@colorado.edu"  
	To = "bayu9537@colorado.edu"  
	file_name = "webcam.jpg"  
     
	server = smtplib.SMTP('smtp.gmail.com',587) 
	server.starttls() 
	server.login("bayu9537@colorado.edu",'Bjjl@1234@')
      
	main_msg = email.MIMEMultipart.MIMEMultipart()  
      

	text_msg = email.MIMEText.MIMEText("Alert!")  
	main_msg.attach(text_msg)  
      
	contype = 'application/octet-stream'  
	maintype, subtype = contype.split('/', 1)  
      

	data = open(file_name, 'rb')  
	file_msg = email.MIMEBase.MIMEBase(maintype, subtype)  
	file_msg.set_payload(data.read( ))  
	data.close( )  
	email.Encoders.encode_base64(file_msg)  
      
	basename = os.path.basename(file_name)  
	file_msg.add_header('Content-Disposition', 'attachment', filename = basename)  
	main_msg.attach(file_msg)  

	main_msg['From'] = From  
	main_msg['To'] = To  
	main_msg['Subject'] = "Alert image "  
	main_msg['Date'] = email.Utils.formatdate( )  
      
 
	fullText = main_msg.as_string( )  
      

	try:  
		server.sendmail(From, To, fullText)  
	finally:  
		server.quit()  

	

