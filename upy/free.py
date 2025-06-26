
import os

stat = os.statvfs('/flash')

block_size = stat[0]      # f_bsize
total_blocks = stat[2]    # f_blocks
free_blocks = stat[3]     # f_bfree

total_bytes = block_size * total_blocks
free_bytes = block_size * free_blocks
used_bytes = total_bytes - free_bytes

used_percent = (used_bytes / total_bytes) * 100

print("Total Flash Size: {:,} bytes".format(total_bytes))
print("Free Flash Space: {:,} bytes".format(free_bytes))
print("Used Flash Space: {:,} bytes ({:.2f}%)".format(used_bytes, used_percent))
