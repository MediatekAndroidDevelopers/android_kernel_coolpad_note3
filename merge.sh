echo "Enter tag one ( won't merge this commit)"
read tag1
echo Enter tag two
read tag2
for i in $(git log --pretty=oneline $tag1...$tag2 | awk '{a[i++]=$1} END {for (j=i-1; j>=0;) print 'a[j--]' }')
do
	git cherry-pick $i
done
