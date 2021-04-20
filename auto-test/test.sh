$ie = New-Object -com 'InternetExplorer.Application'
$ie.Navigate(https://www.baidu.com)
$ie.visible = $true

do { sleep 5 }
while ( $ie.busy )

$doc = $ie.document

$myradios = $doc.getElementsByTagName('input') | ? {$_.type -eq 'radio' -and $_.name -eq 'classcode'}
$x = 0 #specific ridio button 
$myradios[$x].setActive()
$myradios[$x].click()
