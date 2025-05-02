
## 📘 Week 1 – Internet Architecture and Layering (Fully Corrected)

---

### **Q1. What layer of the Internet architecture includes TCP and UDP?**
**Answer:** ✅ Transport  
**Explanation:** TCP and UDP operate at the transport layer, enabling end-to-end communication between applications on different hosts.

---

### **Q2. What layer of the Internet architecture includes FTP and HTTP?**
**Answer:** ✅ Application  
**Explanation:** FTP and HTTP are application-layer protocols that use transport-layer protocols like TCP to send and receive data.

---

### **Q3. What layer of the Internet architecture employs IP?**
**Answer:** ✅ Network  
**Explanation:** The Internet Protocol (IP) is a network-layer protocol responsible for addressing and routing packets between hosts.

---

### **Q4. Which of these is not a characteristic of the service that TCP provides to higher-level applications?**
**Answer:** ✅ Unreliable transmission  
**Explanation:** TCP provides reliable transmission, ensuring in-order delivery, retransmission of lost packets, and error checking. Unreliable transmission is a characteristic of UDP.

---

### **Q5. When you browse google.com, your computer translates the domain name to an IP address through which service?**
**Answer:** ✅ DNS  
**Explanation:** DNS (Domain Name System) translates domain names like google.com into IP addresses needed for routing traffic.

---

### **Q6. Which of the following is true of circuit switching?**
**Answer:** ✅ It makes reserving bandwidth for particular flows easier  
**Explanation:** Circuit switching reserves a dedicated path and bandwidth for the duration of a session, which simplifies flow control and timing.

---

### **Q7. Which of the following is true of packet switching?**
**Answer:** ✅ It can enable higher utilization of bandwidth than circuit switching  
**Explanation:** Packet switching allows many flows to share the same network resources, leading to more efficient bandwidth use.

---

### **Q8. Each protocol in the Internet’s layered architecture…**
**Answer:** ✅ Builds on the service interface of the protocol in the layer immediately below it  
**Explanation:** Protocols rely on the functionality provided by the layer beneath, forming a modular stack with abstracted responsibilities.

---

### **Q9. When a packet arrives at a switch or router, it:**
**Answer:** ✅ May wait in a queue until it can be transmitted on an output interface and may be dropped without being transmitted.  
**Explanation:** Routers and switches use queues when interfaces are congested. If queues fill up, packets may be dropped.

---

### **Q10. In the context of the Internet, an autonomous system is:**
**Answer:** ✅ A network or networks under the control of a single administrative entity  
**Explanation:** An AS is a collection of IP routing prefixes under one administration that presents a common routing policy.

---

✅ **Week 1 – Complete and Verified**
---
## 📘 Week 2 – Forwarding and Routing (Fully Corrected)

---

### **Q1. Packet forwarding is carried out in the:**
**Answer:** ✅ Data Plane  
**Explanation:** The data plane handles fast packet forwarding using the forwarding table, while the control plane computes routing decisions.

---

### **Q2. What port is used for destination address 10101111?**
**Answer:** ✅ Port 4  
**Explanation:** Longest prefix match:
- 101***** → matches 10101111 (7 bits) → length 3  
- 10101*** → matches 10101111 (8 bits) → length 5  
- 10101011 → matches 10101111 exactly (8 bits) → length 8 ✅  
- ✅ Longest match: 10101011 → **Port 4**

---

### **Q3. If packet arrival speed exceeds processing speed, this is handled by:**
**Answer:** ✅ Buffers  
**Explanation:** Buffers temporarily store packets when arrival rate exceeds processing rate, preventing immediate loss.

---

### **Q4. Head-of-line blocking can be avoided by:**
**Answer:** ✅ Virtual output queues  
**Explanation:** VOQ isolates queues per output port at each input, eliminating HOL blocking caused by shared input buffers.

---

### **Q5. What information is created by the control plane and used by the data plane?**
**Answer:** ✅ Forwarding tables  
**Explanation:** The control plane computes routing paths and installs them into forwarding tables used by the data plane.

---

### **Q6. Border Gateway Protocol (BGP) uses:**
**Answer:** ✅ Path vector routing  
**Explanation:** BGP uses AS-paths in routing updates, maintaining full path history to avoid loops.

---

### **Q7. First node explored by Dijkstra’s algorithm from Node A?**
**Answer:** ✅ Node B  
**Explanation:** Dijkstra's algorithm selects the node with the smallest tentative distance from the start — initially Node B.

---

### **Q8. Next node explored by Dijkstra’s algorithm?**
**Answer:** ✅ Node C  
**Explanation:** After Node B, the node with the next lowest tentative distance (updated from B) is chosen — Node C.

---

### **Q9. Next node explored after that?**
**Answer:** ✅ Node H  
**Explanation:** As distances are updated, Node H becomes the next closest unvisited node.

---

### **Q10. In a path vector protocol, what properties can router R be sure of?**
**Answer:** ✅ Its selected path to the destination does not contain loops  
**Explanation:** Path vector protocols include the full AS path in advertisements, allowing loop detection.

---

### **Q11. In a link state protocol, what can router R1 compute with only its LSDB?**
**Answer:** ✅ All of the above  
- A loop-free path from R1 to each destination  
- The cost of an optimal path from R1 to each destination  
- A loop-free path between any two other routers  
- The cost of an optimal path between any two other routers  
**Explanation:** Link-state protocols flood complete network topology, enabling full computation from any point.

---

### **Q12. In a distance vector protocol, what can R1 compute?**
**Answer:** ✅ The cost of an optimal path from R1 to each destination  
**Explanation:** Distance vector protocols store only vector distances to each destination. Loop-free paths to *others* can't be inferred.

---

### **Q13. In a path vector protocol, what can R1 compute?**
**Answer:** ✅ All of the above  
**Explanation:** While R1 only stores paths to destinations via itself, one can assemble paths between other nodes through R1 and prune loops.

---

✅ **Week 2 – Complete and Corrected**


---

# 📘 Week 3 – IP Forwarding and Addressing (Fully Explained)

---

### **Q1. The maximum length of packet in a standard IPv4 communication is:**
✅ 64 kilobytes  
**Explanation:**  
IPv4 packets can be up to 65,535 bytes (or 64 KB), including header and data, as limited by the 16-bit total length field in the IPv4 header.

---

### **Q2. If a network needs 50,000 host addresses, what IPv4 prefix length will provide enough addresses while minimizing unused ones?**
✅ 16  
**Explanation:**  
A /16 prefix gives \( 2^{16} = 65,536 \) IP addresses, which is the smallest subnet large enough to support 50,000 hosts.

---

### **Q3. If networks were constrained to use the historical “classful” method of IP allocation, which of the following two networks would use the same class?**
✅ A network with 300 hosts and a network with 50,000 hosts  
**Explanation:**  
Under classful addressing:
- **Class C** supports up to 254 hosts
- **Class B** supports up to 65,534 hosts
- **Class A** supports up to ~16 million hosts

A network with 300 hosts and another with 50,000 hosts **both require more than 254**, so they fall under **Class B**.

---

### **Q4. Where would packets destined to 128.174.142.33 be forwarded?**
✅ *Interface 1 -> based on longest prefix match*  
**Explanation:**  
The router uses the most specific match (i.e., the longest prefix) to determine the correct forwarding interface.

---

### **Q5. Where would packets destined to 128.174.142.203 be forwarded?**
✅ *R1 -> Interface based on longest prefix match*  
**Explanation:**  
The router selects the forwarding entry whose prefix matches the destination address with the most bits.

---

### **Q6. Where would packets destined to 128.174.149.33 be forwarded?**
✅ * R3 -> Interface based on longest prefix match*  
**Explanation:**  
Longest prefix match ensures the most specific route is chosen to forward the packet.

---

### **Q7. The size of the IPv6 address space is:**
✅ 128 bits  
**Explanation:**  
IPv6 uses 128-bit addresses, supporting 3.4×10³⁸ unique IPs — vastly more than IPv4’s 32-bit system.

---

### **Q8. Which protocol handles control messages between IP modules (like errors, TTL expiry)?**
✅ ICMP  
**Explanation:**  
The Internet Control Message Protocol (ICMP) is used by IP for diagnostics and error reporting (e.g., ping, traceroute, unreachable messages).

---

### **Q9. In a strict IP-layered router, which headers are treated as opaque (not parsed)?**  
✅ TCP header  
✅ HTTP request or response  
**Explanation:**  
The IP layer forwards the payload without examining TCP or application-layer data — those are opaque to IP.

---

### **Q10. Which headers are *not* visible to the IP layer because they’re stripped off beforehand?**  
✅ Ethernet header  
**Explanation:**  
The Ethernet (data-link) layer removes its own header before delivering the packet to the IP layer. IP sees only network-layer and above.

---

### **Q11. In DHCP soft state, which of the following are true?**  
✅ The DHCP server is responsible for terminating the lease  
✅ The client is responsible for renewing the lease while in use  
**Explanation:**  
DHCP uses soft-state management where clients must renew leases before expiry. Servers can reclaim unused addresses when leases expire.

---

### **Q12. Which MAC addresses might appear as destinations as the packet goes from S → D?**  
✅ MAC address of interface i2  
✅ MAC address of interface i4  
✅ MAC address of interface i6  
**Explanation:**  
Each Ethernet segment has a different destination MAC — the next hop’s MAC at each step. The destination MAC changes per hop.

---

# 📘 Week 5 Quiz – Interdomain Routing and BGP (Study Guide Format)

---

## Question 1
**Q:** What protocol does a router use to learn routes to other autonomous systems (ASes)?  
✅ **Answer:** eBGP

**Explanation:**  
External BGP (eBGP) is used to exchange routing information between routers in different autonomous systems. It advertises reachability to IP prefixes across AS boundaries, forming the basis of interdomain routing on the Internet.

---

## Question 2
**Q:** In the given topology and policy constraints, which path would AS 100 use to reach AS 105?  
**Answer:** `AS 100 → AS 103 → AS 102 → AS 105`  
**Explanation:**  
This path respects all AS policies:
- AS 100 does not allow transit through itself.
- AS 103 cannot go through AS 104.
- AS 102 requires routes to go through AS 105.
This is the shortest valid policy-compliant path.
---

## Question 3
**Q:** Which route would AS 101 select to reach prefix 172.110.32.0/21?  
✅ **Answer:** AS 101 → AS 104 → AS 102 → AS 105

**Explanation:**  
Given that AS 101 prefers routes through AS 104 and this path satisfies the policy constraints, BGP would select it even if another path has a shorter AS-path length. Local policy (e.g., higher LocalPref) overrides other considerations.

---

## Question 4
**Q:** Which mechanism is best for AS 100 to implement its policy of not being used as a transit AS?  
✅ **Answer:** Export filter

**Explanation:**  
Export filters control which prefixes are advertised to which neighbors. AS 100 can prevent advertising learned routes from one peer/provider to another, thus preventing itself from being used as a transit AS.

---

## Question 5
**Q:** Which BGP mechanism should AS 101 use to implement its preference for AS 104 paths?  
✅ **Answer:** Import filter with LocalPref

**Explanation:**  
Local Preference is a BGP attribute used within an AS to rank route choices. AS 101 can set a higher LocalPref for routes received from AS 104 to prefer them over other alternatives.

---

## Question 6
**Q:** Which of the following statements about BGP is true?  
✅ **Answer:** A BGP router can send a withdrawal message if a route is no longer available

**Explanation:**  
BGP uses UPDATE messages to either advertise new routes or withdraw existing ones. A withdrawal tells neighboring routers that a prefix is no longer reachable.

---

## Question 7
**Q:** Which of the following are true about iBGP and interdomain routing?  
✅ **Answer:** ASes do not want to expose internal topology; iBGP full mesh requires O(n²) sessions

**Explanation:**  
iBGP routers within an AS must either form a full mesh or use route reflectors. ASes also hide their internal topologies externally, sharing only summarized information like AS-paths and next-hops.

---

## Question 8
**Q:** What type of routing algorithm does BGP use?  
✅ **Answer:** Path vector

**Explanation:**  
BGP is a path vector protocol where each advertised route includes the sequence of ASes it traverses. This helps prevent routing loops and facilitates policy-based routing decisions.

---

## Question 9
**Q:** What is meant by “hot potato” routing in interdomain routing?  
✅ **Answer:** Each AS hands off traffic to the closest exit point

**Explanation:**  
In hot potato routing, each AS attempts to offload outbound traffic to another AS as quickly as possible to minimize its own internal network costs.

---

### Question 10
**Q:** Why might traceroute fail to provide a complete picture of a path?

✅ Some devices along the path are not running IP  
✅ Routers may be configured to not respond with ICMP messages  
✅ End-hosts may be configured to not respond with ICMP messages  
✅ It provides the sender with information about the forward path from source to destination, but not the return path from destination back to source  

**Explanation:**  
Traceroute relies on ICMP Time Exceeded responses from routers along the path. Several factors can cause it to fail or produce incomplete results:
- Some intermediate devices may not process IP (e.g., Layer 2 bridges or MPLS routers).
- ICMP responses may be blocked or deprioritized by routers or end-hosts.
- Traceroute only measures the *forward path*, and the return path may differ due to asymmetric routing, which is common in interdomain routing.

---

✅ **Week 5 – Interdomain Routing and BGP Completed!**

---

# 📘 Week 6 Quiz – TCP and Congestion Control (Study Guide Format)

---

## Question 1
**Q:** Which services are provided by UDP?  
✅ **Answer:** Unreliable datagram delivery, multiplexing multiple connections

**Explanation:**  
UDP offers lightweight, connectionless packet delivery without guarantees of reliability or ordering. It uses port numbers to manage multiple concurrent conversations.

---

## Question 2
**Q:** Which services are provided by TCP?  
✅ **Answer:** Reliable byte stream data delivery, flow control, congestion control

**Explanation:**  
TCP ensures reliable, in-order data delivery. It provides mechanisms like acknowledgments, retransmissions, flow control (window size management), and congestion control (adjusting sending rates).

---

## Question 3
**Q:** TCP is:  
✅ **Answer:** Full-duplex

**Explanation:**  
TCP connections allow simultaneous, independent data flows in both directions. Both ends maintain buffers for sending and receiving.

---

## Question 4
**Q:** If sender/receiver window size is 8 (SWS = RWS = 8), what’s the minimum number of bits needed for sequence numbers?  
✅ **Answer:** 4 bits

**Explanation:**  
Sequence number space must be at least 2 × window size to avoid ambiguity.  
\(2 × 8 = 16\), and \(\log_2 16 = 4\) bits.

---

## Question 5
**Q:** How does TCP's congestion window (cwnd) behave during slow start before a timeout occurs?  
✅ **Answer:** It is multiplied by 2

**Explanation:**  
During slow start, TCP increases cwnd exponentially by doubling it every RTT until a threshold or packet loss is detected.

---

## Question 6
**Q:** You have a network path with links: 4 Gbps, 75 Mbps, and 80 Mbps. What is the end-to-end throughput?  
✅ **Answer:** 75 Mbps

**Explanation:**  
The slowest link in the path (the bottleneck) determines maximum throughput. Here, it's the 75 Mbps link.

---

## Question 7
**Q:** What is the propagation delay across a 435 km fiber-optic path?  
✅ **Answer:** 4.39 ms

**Explanation:**  
Speed of light in fiber ≈ \(2 × 10^8\) m/s.  
Propagation delay = Distance / Speed = \(435,000 / 2 × 10^8\) = 2.175 ms one-way → ≈4.39 ms RTT.

---

## Question 8
**Q:** What’s the max throughput using stop-and-wait with 1000-byte packets and RTT = 8.78 ms?  
✅ **Answer:** ~228 KB/s  
**Explanation:**  
Stop-and-Wait allows only one packet to be in flight per RTT.  
Given the RTT ≈ 4.501 ms (including transmission time) and packet size 1 KB:  
Throughput = 1 KB / 0.00439 sec ≈ 222 KB/sec.  
(Reference: 2.5 Reliable Transmission — Computer Networks: A Systems Approach【401†source】)

**Excel Tip:**  
```excel
=1/0.00439
```

---

## Question 9
**Q:** What is the minimum sliding window size to fully utilize a 75 Mbps link with 4.39 ms RTT?  
✅ **Answer:** 41 KB

**Explanation:**  
##### Bandwidth = 75 Mbps = 75 × 10⁶ bits/sec
##### RTT = 4.39 ms = 0.00439 sec
##### Step 1: Bandwidth in Bytes/sec:  75,000,000÷8=9,375,000 Bytes/sec
##### Step 2: Bandwidth-Delay Product (BDP): BDP=9,375,000×0.00439=41,156.25 ##### Bytes Step 3: Convert to KB: 41,156.25÷1024≈40.2 KB
---

## Question 10
**Q:** A TCP connection has throughput = 80 Mbps, RTT = 10 ms, MTU = 1000 bytes. How many packets are in flight?  
✅ **Answer:** 100 packets

**Explanation:**  
BDP = 80 Mbps × 0.01s = 800,000 bits = 100,000 bytes.  
Packets in flight = 100,000 / 1000 = 100 packets.

---

## Question 11
**Q:** Two long-lived TCP flows share a 100 Mbps link. What is their expected long-term throughput?  
✅ **Answer:** About 50 Mbps each

**Explanation:**  
TCP congestion control aims to fairly share available bandwidth across competing flows, assuming similar RTTs and no major losses.

---

## Question 12
**Q:** What mechanism allows this fair sharing?  
✅ **Answer:** Additive Increase, Multiplicative Decrease (AIMD)

**Explanation:**  
TCP slowly increases its congestion window (additive) and cuts it in half when detecting loss (multiplicative), ensuring fairness.

---

## Question 13
**Q:** In practice, TCP fairness is imperfect because:  
✅ **Answer:** Senders may oscillate around fairness; different RTTs cause unequal rates

**Explanation:**  
Flows with shorter RTTs ramp up their window size faster.  
This causes throughput unfairness across flows with differing network conditions.

---

✅ **Week 6 – TCP and Congestion Control Completed!**

---


# 📚 Week 8 Quiz — DNS and DNS Security (Fully Corrected)

---

## Question 1
**The mapping from DNS names to IP addresses is:**

✅ Many-to-many

**Explanation:**  
DNS names can resolve to multiple IP addresses (for redundancy, load balancing),  
and a single IP address can map to multiple DNS names (e.g., virtual hosting).  
This flexibility helps scalability and robustness.

---

## Question 2
**When a host wants the IP address of a hostname, where is the query first sent?**

✅ Local DNS server

**Explanation:**  
The host first queries its configured local DNS resolver (often provided by its ISP or enterprise network).  
This resolver handles the recursive lookup if necessary.

---

## Question 3
**Which of the following is true of DoS attacks?**

✅ DoS floods targets with traffic; sources in DDoS are often compromised

**Explanation:**  
A DoS overwhelms a server with traffic from a single or few sources.  
A DDoS uses a network of compromised devices (botnet) to flood the target, exhausting its resources.

---

## Question 4
**When a host moves networks and gets a new IP, what DNS servers MUST be updated?**

✅ Authoritative DNS server

**Explanation:**  
The authoritative server is responsible for mapping the domain name to the correct IP address.  
It must be updated when the host’s IP changes to ensure correct resolution.

---

## Question 5
**With DNSSEC, which DNS servers MUST be updated after IP change?**

✅ Authoritative DNS server

**Explanation:**  
Even with DNSSEC (which adds authentication),  
the authoritative server must still be updated to reflect new IP addresses.

---

## Question 6
**How can a host ensure clients find it within 5 minutes after changing IP?**

✅ Ensure the TTL is less than 5 minutes

**Explanation:**  
TTL (Time To Live) controls caching duration.  
If TTL exceeds 5 minutes, old IP addresses might still be cached.  
Setting TTL < 5 min ensures quicker cache refreshes.

---

## Question 7
**How does DNS reduce the number of client queries?**

✅ Caching

**Explanation:**  
DNS caches responses at multiple layers:  
browser, OS, local resolver, and intermediate resolvers.  
This dramatically reduces query load and lookup latency.

---

## Question 8
**If nothing is cached, how many servers are involved in resolving `host.whatever.org`?**

✅ 4 servers — local, root, TLD, and authoritative

**Explanation:**  
The resolution path is:  
1. Local DNS resolver →  
2. Root server →  
3. TLD server (`.org`) →  
4. Authoritative server for `whatever.org`.

---

## Question 9
**With caching, what’s the minimum number of servers that may be contacted?**

✅ 0 — local DNS server

**Explanation:**  
If the local DNS server has a valid, unexpired cached entry,  
it can respond immediately without contacting external servers.
The client may have cached the name

---

## Question 10
**If DNSSEC is used, what can an eavesdropper see?**

✅ Queried name, IP address in response, Bob's digital signature, Bobs public key

**Explanation:**  
DNSSEC ensures data authenticity but does not encrypt queries or responses.  
Thus, an observer can see domain names, IP addresses, and attached signatures —  
but not secret signing keys.

---

## Question 11
**In DNSSEC, authenticity relies on...?**

✅ Authoritative server's signature, public keys, root key signature

**Explanation:**  
The resolver verifies DNSSEC signatures using public keys,  
with a chain of trust anchored at the DNS root’s trusted key.

---

## Question 12
**In a DNS reflection/amplification attack, what source IP does the attacker use?**

✅ Victim’s IP (2.0.0.5)

**Explanation:**  
The attacker spoofs the victim’s IP address,  
causing DNS servers to send responses to the victim,  
overwhelming it.

---

## Question 13
**In the scenario of the previous question, for most or perhaps all packets it sends to conduct the attack, the attacker will use what IP destination address?**

✅ Other (e.g., open recursive DNS servers on the Internet)

**Explanation:**  
In a DNS reflection/amplification attack, the attacker spoofs the source IP address to be the victim’s (e.g., 2.0.0.5) and sends DNS queries **to intermediate DNS servers**. These DNS servers, thinking the query came from the victim, send their large responses to the victim.  
So the **destination address** of the attacker’s outgoing packets is not the victim, but rather **any open DNS server** that can be used as a reflector. These reflectors can be scattered across the Internet and are often unrelated to the victim’s or attacker’s actual DNS infrastructure.

---

✅ **End of Week 8: DNS and DNSSEC Quiz!**

---



# 📚 Week 10 — Data Center Applications, Traffic, and Physical Structure (Fully Corrected)

---

### **Q1. In an all-to-all traffic pattern, which workload likely has an all-to-all pattern?**
**Answer:**  
✔️ Map tasks sending results to reduce tasks in a MapReduce deployment

**Explanation:**  
In a MapReduce deployment where each node acts as both mapper and reducer,  
mappers must send outputs to **all reducers**, causing an all-to-all traffic pattern (complete graph).  
This matches the graph theory definition of a complete bipartite graph becoming all-to-all when merged.

---

### **Q2. What can you conclude from the PDF (probability distribution function) of flow sizes?**
**Answer:**  
✔️ 70% of flows are at most 1000 kb in size

**Explanation:**  
Two things: (1) The way to read probability distribution (or density) functions is this: the area under the curve between two points on the x-axis gives the likelihood that the quantity on the x-axis lies between those two bounds. (2) It's worth noting that there's a difference between between the number of tlows and the number of bvtes - 10% of tlows is not the same as /U♎ Of bvtes. Because I there can be a huge variance in ﬂow sizes, a small fraction of ﬂows might be responsible for most of the bytes in the network.
The PDF indicates the relative frequency of flow sizes. If 70% fall under 1000 KB, it means most flows are short, impacting how resources should be optimized.

---

### **Q3. What can you conclude from the CDF (cumulative distribution function) of flow sizes?**
**Answer:**  
✔️ 70% of flows are at most 1000 kilobytes in size

**Explanation:**  
In a CDF, the y-axis value at x = 1000KB shows the fraction of flows ≤ 1000KB.  
Thus, a point at (1000, 0.7) means **70% of flows** are 1000KB or smaller.

---

### **Q4. Large pool of servers, 10ms or 1s response times. Probability job J (100 requests) takes 1s?**
**Answer:**  
✔️ 0.0952

**Explanation:**  
If each request takes 1 second with 0.001 probability,  
the probability **all 100 requests** are fast is:  
(0.999)^100 ≈ 0.9048.  
Thus, probability at least one request is slow (J takes 1s) is:  
**1 - 0.9048 ≈ 0.0952**. 
P(job takes 1s)=1−P(all 100 requests finish in 10ms)=1−(0.999) 
100
 ≈0.0952

**Excel Tip:**  
`=1-(0.999^100)`

---

### **Q5. In the previous setting, what is the median completion time for job J?**
**Answer:**  
✔️ 10 milliseconds

**Explanation:**  
Even though 1-second responses happen, they are rare (less than 10%).  
Thus, **more than 50% of the time**, the batch finishes with 10ms responses.

---

### **Q6. What is the mean completion time of a single request?**
**Answer:**  
✔️ 10.99 milliseconds

**Explanation:**  
Mean = (10ms × 0.999) + (1000ms × 0.001)  
= 9.99 + 1  
= **10.99ms**
E[X]=(10×0.999)+(1000×0.001)=9.99+1=10.99 ms

**Excel Tip:**  
`=(10*0.999)+(1000*0.001)`

---

### **Q7. What is the mean completion time for job J (100 parallel requests)?**
**Answer:**  
✔️ `104.25 ms`

**Explanation:**  
Job J finishes only after all 100 requests are done. So it finishes in 10ms only if all requests are fast.

Let:
- Fast response time = 10 ms  
- Slow response time = 1000 ms  
- Probability of fast response = 0.999  
- Probability of slow response = 0.001  
- “...request-response time is random, with the following distribution:
10ms with probability 0.999, but 1 second (1000ms) with the remaining probability (0.001).”

 - So, each individual request has:

- a 99.9% chance of taking 10ms, and

- a 0.1% chance of taking 1000ms.
**Step 1: Probability all 100 are fast:**

\[
P({all fast}) = 0.999^{100} => approx 0.9048
\]

**Step 2: Probability at least one is slow (J takes 1s):**

\[
P({any slow}) = 1 - 0.999^{100} => approx 0.0952
\]

**Step 3: Expected completion time (E[J]):**

\[
E[J] = (0.9048 * 10) + (0.0952 * 1000) = 9.048 + 95.2 = {104.25 { ms}}
\]


Right. Although the full job J and an individual request have the same median completion time, J's mean is much higher. Means are affected significantly by outliers in the tail of the distribution.

**Excel Tip:**
=(0.999^100)*10 + (1 - 0.999^100)*1000

---

### **Q8. What causes correlated (non-independent) request times in real systems?**
**Answer:**  
✔️ All of the above

**Explanation:**  
- Servers sharing hardware (CPU, disk, etc.)  
- Congestion in network switches or links  
- Shared backend services like databases  
All these introduce positive correlation between otherwise "independent" service times.

---

### **Q9. Benefits of larger clusters connected with high bandwidth?**
**Answer:**  
✔️ All of the above

**Explanation:**  
- Greater application scaling (across racks, rows)  
- Improved packing efficiency (job placement, utilization)  
- Fault tolerance across failure zones  
High-bandwidth clusters enable all three advantages simultaneously.

---

### **Q10. 40 servers with 40Gbps NICs. Will the non-blocking network handle this without congestion?**
**Answer:**  
✔️ Maybe (not enough information)

**Explanation:**  
It depends on **traffic pattern**.  
If all servers send to different destinations ➔ fine.  
If all servers target the same destination ➔ **incast congestion** occurs.

---

### **Q11. Non-blocking leaf-spine using k-port switches. Max servers supported?**
**Answer:**  
✔️ k²/2 servers

**Explanation:**  
Half of the ports are used for uplinks, half for server downlinks.  
Thus:  
**Max Servers = k × (k/2) / 2 = k²/2**



**Excel Tip:**  
`=k^2/2`

---

### **Q12. How many servers does a fat-tree built with k=6 switches support?**
**Answer:**  
✔️ 54 servers

**Explanation:**  
Fat-tree formula: **(k³)/4 servers**.  
Substituting k = 6:  
(6³)/4 = 216/4 = **54 servers**.

**Excel Tip:**  
`=(k^3)/4`

---

### **Q13. Is a fat-tree topology a tree?**
**Answer:**  
✔️ No

**Explanation:**  
Fat-trees contain **loops and redundant paths** for fault tolerance and load balancing.  
Thus, they are **not loop-free**, unlike classical trees.

---

### **Q14. How many minimal paths exist from a top-layer switch to any server?**
**Answer:**  
✔️ 1

**Explanation:**  
In fat-trees, there is a **single shortest path** from a top-layer (core) switch to a server.  
No multiple minimal-length options downward.

---

### **Q15. Failure of aggregation switch — are two topologies identical?**
**Answer:**  
✔️ No

**Explanation:**  
Failure in certain switches can disconnect a pod in one topology but not the other.  
Subtle topology differences lead to different fault tolerance characteristics.

---

### **Q16. Cable scaling with port count k in fat-trees?**
**Answer:**  
✔️ k³

**Explanation:**  
Cable count grows like **k³** in fat-trees (because switch interconnections explode).  
This makes cabling cost and complexity massive at scale.

**Excel Tip:**  
`=k^3`

---

✅ End of Week 10 — Data Center Applications, Traffic, and Physical Structure Quiz!


---
# 📚 Week 11 — Cloud Routing and Congestion Control (Fully Corrected)

## Q1. In a fat-tree with 32 switch-switch links, how many links are used by a spanning tree?
**Answer:** ✔️ 19

**Explanation:**  
A spanning tree connects all switches with the minimum number of links.  
For n switches, spanning tree uses n - 1 links.  
Here, 20 switches ⇒ 19 links.

**Excel Tip:**  
`=number_of_switches - 1`

---

## Q2. Available bandwidth between the left-most pod to another pod (failed switches)?
**Answer:** ✔️ 2 units

**Explanation:**  
Each pod retains two functioning links to the core.  
Thus, between pods, the available capacity = 2 units even after failures.

---

## Q3. How does TRILL enable the use of all links?
**Answer:** ✔️ By running a link-state protocol between switches

**Explanation:**  
TRILL (Transparent Interconnection of Lots of Links) uses link-state routing like OSPF.  
This allows switches to discover full network topology and select multiple paths intelligently.

---

## Q4. In BGP, path announcements contain:
**Answer:** ✔️ The autonomous system-level path from node to destination

**Explanation:**  
BGP includes the full AS-path in its announcements to prevent loops.  
It doesn't reveal the full internal network topology.

---

## Q5. How does ECMP avoid packet reordering?
**Answer:** ✔️ By consistently using the same path for packets of the same flow

**Explanation:**  
ECMP (Equal-Cost Multipath Routing) hashes packet headers to select paths.  
Thus, all packets of a flow stick to a single path, preventing reordering.

---

## Q6. How many ECMP paths between a server pair in a fat-tree built with k=10?
**Answer:** ✔️ 25

**Explanation:**  
The number of ECMP paths = (k/2)² = (10/2)² = 25 paths.

**Excel Tip:**  
`=(k/2)^2`

---

## Q7. How does CONGA forward traffic on multiple paths?
**Answer:** ✔️ Congestion is monitored, and least congested paths are chosen

**Explanation:**  
CONGA monitors congestion on each path and dynamically routes flows to the least congested one, improving load balancing.

---

## Q8. What are flowlets?
**Answer:** ✔️ Flowlets are packet-groups in the same flow, separated by large enough gaps

**Explanation:**  
A flowlet is a subset of a flow’s packets, divided when inter-packet gaps are large.  
Routing flowlets independently reduces reordering risk.

---

## Q9. How do containers differ from virtual machines?
**Answer:** ✔️ All of the above

**Explanation:**  
Containers share the host OS kernel and are faster, lighter, and more portable compared to VMs which run separate OS instances.

---

## Q10. Processing time per packet at 100 Gbps with 100-Byte packets?
**Answer:** ✔️ 8 nanoseconds

**Explanation:**  
Each packet is 800 bits.  
Time = 800 bits / 100 × 10⁹ bps = 8 ns.

**Excel Tip:**  
`=(Packet_Size_Bits)/(Line_Rate_Bps)`

---

## Q11. How does SR-IOV help line-rate packet processing?
**Answer:** ✔️ All of the above

**Explanation:**  
SR-IOV provides direct memory access and assigns NIC resources to VMs, bypassing hypervisors and reducing latency.

---

## Q12. Drawbacks of SR-IOV’s approach?
**Answer:** ✔️ Options B and C but not A

**Explanation:**  
- **B:** Migration is harder because NIC state is tied to VMs.  
- **C:** NIC classifiers are simple and not flexible.  
- **A is false** because SR-IOV achieves near line-rate performance.

---

## Q13. How does Open vSwitch (OVS) optimize packet forwarding?
**Answer:** ✔️ All of the above

**Explanation:**  
OVS optimizes performance by caching flows in the kernel for fast forwarding and using user-space processing only for new flows.

---

## Q14. TCP reaction to packet loss during additive increase phase?
**Answer:** ✔️ X/2

**Explanation:**  
Upon detecting a loss, TCP halves its congestion window:  
New window = X/2.

**Excel Tip:**  
`=X/2`

---

## Q15. What problems arise from TCP’s loss-based congestion control?
**Answer:** ✔️ All of the above

**Explanation:**  
- Loss is a poor congestion signal.  
- Multiplicative decrease can be too aggressive.  
- Queues fill up before loss occurs, inflating latency.

---

## Q16. Propagation delay across 300 meters of fiber (speed = 2c/3)?
**Answer:** ✔️ 1.5 microseconds

**Explanation:**  
Speed = (2/3) × 300,000 km/s = 200,000 km/s.  
Delay = 300m / (200,000 km/s) = 1.5 μs.

**Excel Tip:**  
`=(Distance_meters)/(Speed_meters_per_sec)`

---

## Q17. Delay for a packet queued behind 5 others (9000 bytes each) at 10 Gbps?
**Answer:** ✔️ 36 microseconds

**Explanation:**  
5 × 9000 bytes × 8 bits = 360,000 bits.  
Delay = 360,000 bits / 10 × 10⁹ bps = 36 μs.

**Excel Tip:**  
`=(Packets*Packet_Size_Bytes*8)/(Line_Rate_Bps)`

**Explanation:**  
Each packet is 9000 bytes, and there are 5 packets ahead in the queue.  
So the total number of bits before your packet is:
```
= 5 × 9000 × 8 = 360,000 bits
```
At a link rate of 10 Gbps (10 × 10⁹ bits/sec), the delay is:
```
= 360,000 / (10 × 10^9) = 36 × 10⁻⁶ seconds = 36 microseconds
```

**Excel Tip:**  
To compute this in Excel:
```excel
=(5 * 9000 * 8) / (10 * 10^9)
```
Multiply by 1,000,000 for microseconds:
```excel
=((5 * 9000 * 8) / (10 * 10^9)) * 1000000
```
➡️ Result: 36 μs

---

## Q18. Why does queuing delay pose problems in data centers?
**Answer:** ✔️ Large buffer occupancies inflate latency dramatically

**Explanation:**  
Even microsecond-scale propagation delays are small compared to queuing delays from full buffers, leading to inconsistent latency.

---

## Q19. Reducing TCP timeout to fix incast — what happens?
**Answer:** ✔️ While it helps throughput, latencies can still be large

**Explanation:**  
Lowering TCP timeout helps faster recovery, improving throughput,  
but does not prevent large queues and inflated latency.

---

## Q20. How does DCTCP balance low latency and high throughput?
**Answer:** ✔️ All of the above

**Explanation:**  
- Marks packets early when buffers start to fill  
- Adjusts sending rate proportionally to congestion signals  
- Keeps buffer occupancy small, absorbing bursts efficiently

---

## Q21. If α = 0.4 in DCTCP, new congestion window?
**Answer:** ✔️ 0.8X

**Explanation:**  
DCTCP adjusts cwnd as:  
New cwnd = X × (1 - α/2) = X × (1 - 0.2) = 0.8X.

**Excel Tip:**  
`=X*(1-Alpha/2)`

---

## Q22. What governs setting DCTCP’s congestion marking threshold (K)?
**Answer:** ✔️ Both A and B

**Explanation:**  
- If K is too large ➔ still high buffer occupancy  
- If K is too small ➔ frequent throttling ➔ reduced throughput
---

# 📚 Week 12 Quiz — Software Defined Networking (Fully Corrected)

---

## Question 1
**Key characteristics of software-defined networks include (check all that apply):**

✅ An API interface to the data plane of switches  
✅ A programmable, logically centralized network controller

**Explanation:**  
In SDN, the control plane and data plane are separated.  
An API like OpenFlow enables programmable control over switches.  
A logically centralized controller manages the network globally, simplifying management.

---

## Question 2
**In traditional non-virtualized data centers, moving services between machines can be difficult because (check all that apply):**

✅ IP addresses may need to be reassigned  
✅ Some services need to be kept within a single Layer 2 domain  
✅ The performance of the running service can differ significantly if moved outside its primary rack or cluster

**Explanation:**  
IP prefixes are often tied to specific racks.  
L2 domain constraints (broadcast, multicast) limit migration.  
Performance suffers due to increased latency when services are moved across network domains.

---

## Question 3
**The VL2 paper argues that a large number of relatively small flows is a good match because (check all that apply):**

✅ Dynamically changing routes could theoretically perform better, but fast reaction is difficult  
✅ Random assignment spreads load evenly, helping with small flows

**Explanation:**  
Small flows help randomized load balancing work well.  
Adaptive schemes could be better but reacting to congestion in real time is difficult.

---

## Question 4
**VL2's use of Valiant Load Balancing is an example of:**

✅ Oblivious routing of traffic

**Explanation:**  
In VL2, traffic is first sent randomly to an intermediate switch before the final destination,  
without considering live congestion — thus **oblivious** to network conditions.

---

## Question 5
**Using ECMP in a data center network is an example of:**

✅ Oblivious routing

**Explanation:**  
ECMP selects a path based only on a packet’s header hash.  
It does not dynamically adjust based on congestion, making it oblivious.

---

## Question 6
**CONGA's traffic routing is an example of:**

✅ Adaptive routing

**Explanation:**  
CONGA monitors congestion in the network and dynamically chooses the least congested path for flows, reacting in real time.

---

## Question 7
**With worst-case ECMP hashing, what average data rate will flow A1 → Ad receive?**

✅ 1.25 Gbps

**Explanation:**  
If 8 flows share a single 10 Gbps link,  
each gets 10 / 8 = **1.25 Gbps**.

**Excel Tip:**  
`=10/8`

---

## Question 8
**How many paths for A1 → Ad pass through switch S1?**

✅ 1

**Explanation:**  
Each top-level switch (e.g., spine) provides exactly one shortest path toward the destination server.

---

## Question 9
**Q:** Suppose the flows from A₁ and B₁ happen to go through switch Sᵢ for each i ∈ {1, 2, 3, 4}. Approximately what average data rate will flow A₁ → A₄ receive?

✅ **2.5 Gbps**

**Explanation:**  
This is the **ideal case** in which ECMP (Equal-Cost Multi-Path) hashing results in **perfectly balanced flows**.  
If there are 4 links and A₁ and B₁ both send traffic through each, and everything is perfectly distributed, A₁ gets half of each link’s 5 Gbps capacity:

= (5 Gbps × 4 links) / 2 = 10 Gbps / 2 = 5 Gbps

But since A₁ is only one sender and it shares equally with B₁, it receives:

= 5 Gbps / 2 = 2.5 Gbps

---


---

## Question 10
**Q:** Suppose the flow routing is as in the previous question. But now, B's senders are not running a stable, converged TCP; they are sending a burst of UDP traffic at a fixed rate of 4 Gbps each. Approximately what average data rate will flow A₁ → A₄ receive?

✅ **1 Gbps**

**Explanation:**  
In this case, B's senders are aggressively sending **UDP traffic at 4 Gbps**, which **does not yield** like TCP would under congestion.  
Because each switch port has **5 Gbps of capacity**, and B is sending 4 Gbps on each, A is only left with:
= 5 Gbps - 4 Gbps = 1 Gbps

So A₁’s flow to A₄ is starved down to **1 Gbps** due to B’s non-cooperative behavior.

---

## Question 11
**Why does this case fall outside VL2’s performance isolation guarantee?**

✅ Some flows are not TCP and do not back off under congestion  
✅ The traffic does not conform to the "hose model" (NIC bandwidth constraint)

**Explanation:**  
VL2 assumes congestion control via TCP and traffic within NIC limits.  
UDP bursts violate these assumptions.

---

## Question 12
**How does VL2 enable service mobility?**

✅ Two separate IP spaces (AA for application identity, LA for location), with dynamic translation

**Explanation:**  
Application Address (AA) remains stable across moves,  
while Locator Address (LA) reflects physical location.  
Dynamic translation maintains connectivity.

---

## Question 13
**In VL2, an Application Address (AA) identifies:**

✅ A server or virtual server

**Explanation:**  
An AA identifies a virtual machine or server instance,  
abstracting its location.

---

## Question 14
**In VL2, a Locator Address (LA) identifies:**

✅ A rack or top-of-rack (ToR) switch

**Explanation:**  
The LA identifies where in the data center hierarchy the server is physically connected (rack-level).

---

## Question 15
**When a service moves in VL2:**

✅ Its AA stays the same, but its LA may change

**Explanation:**  
Mobility is achieved because AA remains constant  
even if the physical location (LA) changes.

---

## Question 16
**How many total unidirectional tunnels are needed in the given NVP setup?**

✅ 9,990

**Explanation:**  
There are two tenants:
- One with **10 VMs**: each of these needs 9 unidirectional tunnels to the others → `10 × (10 - 1) = 90`
- One with **100 VMs**: each of these needs 99 unidirectional tunnels to the others → `100 × (100 - 1) = 9,900`

So the total number of **unidirectional tunnels** is:  
`90 + 9,900 = 9,990`

**Excel Tip:**  

=(10*(10-1)) + (100*(100-1))
---

## Question 17
**How many total instances of tenants' virtual networks exist?**

✅ 110

**Explanation:**  
Each physical server hosts its own local instance of the full virtual network,  
leading to 110 network instances across 110 servers.

---

## Question 18
**Can NVP enforce microsegmentation?**

✅ Yes, by dropping unauthorized packets or making remote addresses unreachable

**Explanation:**  
Policy enforcement is built into the logical data path of the virtual switches (Open vSwitch instances).

---

## Question 19
**What does NVP guarantee? (check all that apply):**

✅ Layer 2 network semantics  
✅ Segmentation across tenants  
✅ Location-independent addressing

**Explanation:**  
NVP ensures Layer 2 abstraction across data centers,  
logical tenant isolation,  
and address separation for service mobility.  
It does **not** guarantee uniform performance — that depends on underlying physical network hardware.

---

✅ **This now fully, properly corrects and formats Week 12: Software Defined Networking.**

---

# 📚 Week 13 Quiz — Cloud WAN Connectivity (Fully Corrected)

---

## Question 1
**Q:** You’re provisioning a WAN link connecting two data centers. If you could schedule background traffic without delaying latency-sensitive traffic, what fraction of original bandwidth would be needed?

✅ 0.4

**Explanation:**  
By deferring background traffic and spreading it across time slots,  
only the latency-sensitive traffic (40%) must be transmitted immediately.  
Thus, operating with just 40% of the original link capacity suffices.

---

## Question 2
**Q:** Benefits of having multiple geographically distributed data centers?

✅ All of the above

**Explanation:**  
- Lowers client latency by proximity.  
- Improves data availability and disaster resilience.  
- Balances load and traffic across sites.

---

## Question 3
**Q:** Which scalability technique does Google's B4 use?

✅ It aggregates links as well as traffic flows between pairs of sites.

**Explanation:**  
B4 aggregates multiple flows and uses SDN control to efficiently manage inter-datacenter traffic,  
achieving scalability without overwhelming the control plane.

---

## Question 4
**Q:** How much bandwidth is achieved with link-level fairness?

✅ Red: 1 unit, Blue: 0.5 units, Green: 0.5 units

**Explanation:**  
Each individual flow gets 0.5 units due to link sharing.  
Since Red has two flows, its total throughput is 1 unit.

---

## Question 5
**Q:** What is a TCP cookie?

✅ A cryptographic token included in the client’s request.

**Explanation:**  
TCP cookies authenticate clients early,  
enabling the server to validate a client without needing to maintain state before the handshake is completed.

---

## Question 6
**Q:** Sending 100 KB Chicago → New Delhi. What dominates transmission time?

✅ Even if Internet operated at light speed, at least 40 ms.

**Explanation:**  
Over long distances (thousands of kilometers),  
the speed of light imposes a lower bound on transmission latency.

---

## Question 7
**Q:** When does caching *not* help?

✅ All of the above

**Explanation:**  
Caching is ineffective for:
- Dynamic data (e.g., stock prices)
- Personalized content
- Encrypted communications

---

## Question 8
**Q:** Client-server interaction of 6 round trips without CDN, how long?

✅ 2.4 seconds

**Explanation:**  
Each round trip is 400 ms.  
6 RTTs × 400 ms = 2.4 seconds.

---

## Question 9
**Q:** With CDN close to the client, how long does it take?

✅ 0.55 seconds

**Explanation:**  
Latency to a nearby CDN is much lower (~30 ms per RTT).  
6×30 ms + one server RTT (~370 ms) ≈ 0.55 seconds.

---

## Question 10
**Q:** Can overlay routing help in triangle violation scenario?

✅ Yes

**Explanation:**  
Overlay routing allows intermediate nodes to reroute traffic more efficiently,  
correcting poor BGP paths and reducing overall latency.

---

## Question 11
**Q:** How does 8.8.8.8 achieve low latency?

✅ IP Anycast

**Explanation:**  
Google's 8.8.8.8 uses anycast to route users to the nearest server instance,  
even though the same IP address is advertised globally.

---

## Question 12
**Q:** Factors affecting CDN node choice?

✅ Estimated latency to the client  
✅ Load at CDN locations

**Explanation:**  
Good CDN systems use real-time measurements of network latency and server load,  
rather than only relying on physical distance.

---

## Question 13
**Q:** TTL settings for CDN cache entries?

✅ Short TTL for cluster, longer TTL for CDN resolver

**Explanation:**  
- CDN resolver mapping is stable over time → long TTL.  
- Cluster selection may change dynamically → short TTL needed.

---

## Question 14
**Q:** Why OpenDNS/GoogleDNS can interfere with CDNs?

✅ Because the DNS server may not be geographically close to the client.

**Explanation:**  
When DNS resolvers are remote, CDN systems might assign content nodes poorly,  
leading to suboptimal routing.

---

## Question 15
**Q:** Probability that a 100-request batch takes 1 second if any request does?

✅ ~18%

**Explanation:**  
Each request has a 0.2% chance of taking 1 second and a 99.8% chance of being fast (10ms).  
We compute the probability that **all 100 requests are fast**:
= 0.998^100 ≈ 0.8186

Thus, the probability that **at least one request is slow**, and therefore the batch takes 1 second, is:

= 1 - 0.8186 = 0.1814 → 18.14%
---


---

## Question 16
**Q:** With replication after 10 ms, probability of 3+ replications?

✅ ~0.1%

**Explanation:**  
This models the probability that 3 or more requests take 1 second, using a binomial distribution with:
- n = 100 trials
- p = 0.002 (0.2% slow)

We compute:

P(X ≥ 3) = 1 - P(X=0) - P(X=1) - P(X=2)
= 1 - [BINOM.DIST(0,100,0.002,FALSE)
+ BINOM.DIST(1,100,0.002,FALSE)
+ BINOM.DIST(2,100,0.002,FALSE)]
≈ 1 - (0.8186 + 0.1638 + 0.0165)
≈ 0.0011 → 0.1119%
---


---

## Question 17
**Q:** If 3 requests are delayed, what's the chance the batch still takes 1 second?

✅ ~0.6%

**Explanation:**  
This asks for the chance that **at least one** of 3 slow requests is in the original batch — meaning the entire batch is delayed.  
The chance that a single request is **not** slow is 0.998, so:
P(all 3 requests are fast) = 0.998^3 ≈ 0.994
P(batch is delayed) = 1 - 0.994 ≈ 0.006 → 0.6%

---

## Question 18
**Q:** Overheads of request replication?

✅ All of the above

**Explanation:**  
- Higher server-side load  
- Additional network traffic  
- Increased client computation for managing duplicate responses

---

## Question 19
**Q:** App-level methods to handle TCP incast?

✅  
- Limit response size  
- Space out requests  
- Tolerate missing responses

**Explanation:**  
Each technique reduces pressure on switch buffers and minimizes packet loss and retransmissions.

---

## Question 20
**Q:** If estimated capacity drops below bitrate in ABR streaming?

✅ Request chunk at a reduced bitrate.

**Explanation:**  
Traditional ABR algorithms reduce video quality when network bandwidth estimates decrease,  
preventing rebuffering.

---

## Question 21
**Q:** How does a **buffer-based ABR algorithm** react when the buffer is full?

✅ Requests chunk at a large bit-rate

**Explanation:**  
A high buffer occupancy indicates good available bandwidth,  
encouraging the client to switch to a higher video quality.

---

✅ **End of Week 13 Quiz – Cloud WAN Connectivity.**

---


---

## Appendix: What are Gbps and Mbps?

**Gbps (Gigabits per second)** and **Mbps (Megabits per second)** are units of data transfer rate, commonly used to measure network link speeds.

### 📐 Unit Conversions:

- 1 **bit** = smallest unit of data (either 0 or 1)
- 1 **byte** = 8 bits

### ⚙️ Speeds:

- 1 **Kbps** = 1,000 bits per second
- 1 **Mbps** = 1,000,000 bits per second = \( 10^6 \)
- 1 **Gbps** = 1,000,000,000 bits per second = \( 10^9 \)

So when you see:
- **10 Gbps** line rate → means \( 10 * 10^9 \) bits can be transmitted per second.
- **100 Mbps** line rate → means \( 100 * 10^6 \) bits per second.

### 📊 Excel Conversion Tip:

To convert from Mbps or Gbps to bits per second in Excel:

```excel
= 10 * 10^9   // For 10 Gbps
= 100 * 10^6  // For 100 Mbps
```

This helps calculate delays when dividing total bits by line rate in bits/second.

---
