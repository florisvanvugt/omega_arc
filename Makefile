
default: arc

quit:
	make -C robot kill

kill: # Kill the robot, reasonably gently
	make -C robot kill

doc: documentation.html
	xdg-open documentation.html

documentation.html: readme.md fonts/github-pandoc.css
	pandoc -f markdown -t html readme.md -s -c fonts/github-pandoc.css -o documentation.html

simpleschedule.txt: make_schedule.py
	python make_schedule.py

arc: rob
	python2 run_arc.py

rob:
	make -C robot


clean:
	rm -f *.pyc
	rm -f *~
	rm -f documentation.html
	make -C robot clean
